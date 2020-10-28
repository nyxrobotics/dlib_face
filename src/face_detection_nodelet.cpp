// Copyright 2020 Tatsuro Sakaguchi<tacchan.mello.ioiq@gmail.com>

#include <string>
#include <vector>

#include <dlib/data_io.h>
#include <dlib/dnn.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "dlib_face/nodelet.h"
#include "dlib_face/RectArrayStamped.h"

namespace dlib_face
{
/// mmod detection face network
/// example code: http://dlib.net/dnn_mmod_face_detection_ex.cpp.html
template <std::int32_t num_filters, typename SUBNET>
using con5d = dlib::con<num_filters, 5, 5, 2, 2, SUBNET>;
template <std::int32_t num_filters, typename SUBNET>
using con5 = dlib::con<num_filters, 5, 5, 1, 1, SUBNET>;

template <typename SUBNET>
using downsampler = dlib::relu<
    dlib::affine<con5d<32, dlib::relu<dlib::affine<con5d<32, dlib::relu<dlib::affine<con5d<16, SUBNET>>>>>>>>>;
template <typename SUBNET>
using rcon5 = dlib::relu<dlib::affine<con5<45, SUBNET>>>;

using net_type = dlib::loss_mmod<
    dlib::con<1, 9, 9, 1, 1, rcon5<rcon5<rcon5<downsampler<dlib::input_rgb_image_pyramid<dlib::pyramid_down<6>>>>>>>>;

/**
 * @brief Face detection using dlib mmod detector
 *
 */
class FaceDetectionNodelet : public dlib_face::Nodelet
{
  ros::Publisher faces_pub_;
  image_transport::Publisher image_pub_;
  image_transport::Publisher face_image_pub_;
  image_transport::CameraSubscriber image_sub_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  net_type mmod_face_detector_;

  int queue_size_;
  ros::Time prev_stamp_;
  int hz_;
  bool control_hz_;

  /**
   * @brief Detect faces in the image.
   * Publish all detected faces as RectArrayStamped, cropped largest face image as Image, and overlay faces rect on
   * subscribed image as Image.
   *
   * @param msg       Subscribed image message
   * @param cam_info  Subscribed camera info message
   */
  void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& cam_info)
  {
    if (control_hz_ && 1. / hz_ > (msg->header.stamp - prev_stamp_).toSec())
      return;

    try
    {
      // Convert the image into something opencv can handle
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      // Convert OpenCV image format to Dlib's image format
      dlib::cv_image<dlib::bgr_pixel> image(frame);
      dlib::matrix<dlib::rgb_pixel> dlib_mat;
      dlib::assign_image(dlib_mat, image);

      // Detect faces in the image
      std::vector<dlib::mmod_rect> face_rects = mmod_face_detector_(dlib_mat);

      // Convert faces into message and overlay faces rect on raw image
      cv::Mat face_image;
      dlib_face::RectArrayStamped faces_msg;
      faces_msg.header = msg->header;

      if (!face_rects.empty())
      {
        dlib_face::Rect max_area;
        for (const dlib::mmod_rect& face : face_rects)
        {
          dlib_face::Rect rect;
          rect.left = face.rect.left();
          rect.top = face.rect.top();
          rect.width = face.rect.width();
          rect.height = face.rect.height();

          // Sometimes face detection includes the outside of the area, so it is confined inside the area.
          if (rect.left < 0)
          {
            rect.width += rect.left;
            rect.left = 0;
          }
          if (rect.top < 0)
          {
            rect.height += rect.top;
            rect.top = 0;
          }
          double x2 = face.rect.right();
          double y2 = face.rect.bottom();
          if (x2 >= frame.cols)
          {
            rect.width = frame.cols - rect.left;
          }
          if (y2 >= frame.rows)
          {
            rect.height = frame.rows - rect.top;
          }
          faces_msg.rects.push_back(rect);

          if (max_area.height * max_area.width < rect.height * rect.width)
          {
            max_area = rect;
          }
          cv::rectangle(frame, cv::Point(rect.left, rect.top),
                        cv::Point(rect.left + rect.width, rect.top + rect.height), cv::Scalar(0, 0, 255), 3, CV_AA);
        }
        face_image = frame(cv::Rect(cv::Point(max_area.left, max_area.top),
                                    cv::Point(max_area.left + max_area.width, max_area.top + max_area.height)))
                         .clone();
      }

      // Publish the image.
      sensor_msgs::Image::Ptr out_image =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      image_pub_.publish(out_image);
      faces_pub_.publish(faces_msg);
      if (!face_rects.empty())
      {
        sensor_msgs::Image::Ptr out_face_image =
            cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, face_image).toImageMsg();
        face_image_pub_.publish(out_face_image);
      }
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  void subscribe() final
  {
    image_sub_ = it_->subscribeCamera("image", queue_size_, &FaceDetectionNodelet::imageCallback, this);
  }

  void unsubscribe() final
  {
    image_sub_.shutdown();
  }

public:
  void onInit() final
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("hz", hz_, 10);
    pnh_->param("control_hz", control_hz_, false);
    prev_stamp_ = ros::Time(0, 0);

    image_pub_ = advertiseImage(*pnh_, "image", 1);
    face_image_pub_ = advertiseImage(*pnh_, "face_image", 1);
    faces_pub_ = advertise<dlib_face::RectArrayStamped>(*pnh_, "faces", 1);

    std::string model_path;
    pnh_->param<std::string>("model_path", model_path, ".ros/dlib/models/mmod_human_face_detector.dat");
    dlib::deserialize(model_path) >> mmod_face_detector_;
  }
};
}  // namespace dlib_face

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dlib_face::FaceDetectionNodelet, nodelet::Nodelet);
