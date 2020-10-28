// Copyright 2020 Tatsuro Sakaguchi<tacchan.mello.ioiq@gmail.com>

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <dlib/dnn.h>
#include <dlib/graph_utils.h>
#include <dlib/image_io.h>
#include <dlib/image_processing.h>
#include <dlib/opencv.h>
#include <dlib/string.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/image_encodings.h>

#include "dlib_face/nodelet.h"
#include "dlib_face/RectArrayStamped.h"
#include "dlib_face/SaveFace.h"

namespace dlib_face
{
/// ResNet face recognition
/// example code: http://dlib.net/dnn_face_recognition_ex.cpp.html
template <template <int, template <typename> class, int, typename> class block, int N, template <typename> class BN,
          typename SUBNET>
using residual = dlib::add_prev1<block<N, BN, 1, dlib::tag1<SUBNET>>>;

template <template <int, template <typename> class, int, typename> class block, int N, template <typename> class BN,
          typename SUBNET>
using residual_down =
    dlib::add_prev2<dlib::avg_pool<2, 2, 2, 2, dlib::skip1<dlib::tag2<block<N, BN, 2, dlib::tag1<SUBNET>>>>>>;

template <int N, template <typename> class BN, int stride, typename SUBNET>
using block = BN<dlib::con<N, 3, 3, 1, 1, dlib::relu<BN<dlib::con<N, 3, 3, stride, stride, SUBNET>>>>>;

template <int N, typename SUBNET>
using ares = dlib::relu<residual<block, N, dlib::affine, SUBNET>>;
template <int N, typename SUBNET>
using ares_down = dlib::relu<residual_down<block, N, dlib::affine, SUBNET>>;

template <typename SUBNET>
using alevel0 = ares_down<256, SUBNET>;
template <typename SUBNET>
using alevel1 = ares<256, ares<256, ares_down<256, SUBNET>>>;
template <typename SUBNET>
using alevel2 = ares<128, ares<128, ares_down<128, SUBNET>>>;
template <typename SUBNET>
using alevel3 = ares<64, ares<64, ares<64, ares_down<64, SUBNET>>>>;
template <typename SUBNET>
using alevel4 = ares<32, ares<32, ares<32, SUBNET>>>;

using anet_type = dlib::loss_metric<dlib::fc_no_bias<
    128, dlib::avg_pool_everything<alevel0<alevel1<alevel2<alevel3<alevel4<dlib::max_pool<
             3, 3, 2, 2, dlib::relu<dlib::affine<dlib::con<32, 7, 7, 2, 2, dlib::input_rgb_image_sized<150>>>>>>>>>>>>>;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, dlib_face::RectArrayStamped>
    ApproximateSyncPolicy;

/**
 * @brief k-nearest neighbor classification class
 * Dlib classification such as ChineseWhispers is high calculation cost(O(n^2)).
 *
 */
class Classifier
{
  int k_;
  double threshold_;
  // Collect label samples
  std::vector<std::pair<std::string, dlib::matrix<float, 0, 1>>> samples_;

public:
  Classifier()
  {
  }
  /**
   * @brief Construct a new Classifier object
   *
   * @param target_path Path containing of collect label data
   * @param k Number of nearest candidates
   * @param threshold Distance threshold
   */
  explicit Classifier(const boost::filesystem::path& target_path, int k = 3, double threshold = 0.6)
    : k_(k), threshold_(threshold)
  {
    /// Glob collect label data from target_path
    /// Example of data path
    /// .ros/dlib/face_data
    /// ├── person_name1
    /// │   ├── 000000.dat
    /// │   └── 000001.dat
    /// └── person_name2
    ///     └── 000000.dat
    boost::filesystem::create_directories(target_path);
    boost::filesystem::directory_iterator end;
    for (boost::filesystem::directory_iterator it(target_path); it != end; ++it)
    {
      if (boost::filesystem::is_directory(*it))
      {
        std::string label = it->path().stem().string();
        for (boost::filesystem::directory_iterator cit(it->path()); cit != end; ++cit)
        {
          if (boost::filesystem::is_directory(*cit))
            continue;
          boost::filesystem::path file_path = cit->path();

          dlib::matrix<float, 0, 1> face_description;
          dlib::deserialize(file_path.string()) >> face_description;
          samples_.emplace_back(file_path.parent_path().stem().string(), face_description);
        }
      }
    }
  }

  /**
   * @brief Predict what the new sample belongs to
   *
   * @param target New sample face descriptor
   * @return std::string Person name of the group
   */
  std::string predict(const dlib::matrix<float, 0, 1>& target)
  {
    std::vector<std::pair<double, std::string>> candidates;
    for (const auto& sample : samples_)
    {
      dlib::squared_euclidean_distance sed;
      double distance = sed(sample.second, target);
      if (distance < std::pow(threshold_, 2))
      {
        candidates.emplace_back(distance, sample.first);
      }
    }
    // Near samples are not found
    if (candidates.empty())
      return "unknown";

    std::sort(candidates.begin(), candidates.end());
    int n = k_;
    if (n > candidates.size())
      n = candidates.size();

    // Find the person with most close candidate
    std::unordered_map<std::string, int> votes;
    for (int i = 0; i < n; i++)
    {
      if (votes.count(candidates[i].second) == 0)
        votes.emplace(candidates[i].second, 1);
      else
        votes[candidates[i].second]++;
    }
    return std::max_element(votes.begin(), votes.end(),
                            [](const auto& l, const auto& r) { return l.second < r.second; })
        ->first;
  }
};

/**
 * @brief Face recognition using dlib ResNet recognizer
 *
 */
class FaceRecognitionNodelet : public dlib_face::Nodelet
{
  image_transport::Publisher image_pub_;
  boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> async_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<dlib_face::RectArrayStamped> face_sub_;
  ros::ServiceServer save_face_srv_;

  boost::shared_ptr<image_transport::ImageTransport> it_;

  dlib::shape_predictor shape_predictor_;
  anet_type net_;
  Classifier classifier_;
  dlib::matrix<float, 0, 1> latest_descriptor_;

  int queue_size_;
  ros::Time prev_stamp_;
  int hz_;
  bool control_hz_;
  boost::filesystem::path face_data_path_;

  void faceImageCallback(const sensor_msgs::ImageConstPtr& msg, const dlib_face::RectArrayStampedConstPtr& faces)
  {
    if (control_hz_ && 1. / hz_ > (msg->header.stamp - prev_stamp_).toSec())
      return;

    try
    {
      // Convert the image into something opencv can handle
      cv::Mat frame = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8)->image;

      if (!faces->rects.empty())
      {
        // Convert OpenCV image format to Dlib's image format
        dlib::cv_image<dlib::bgr_pixel> image(frame);
        dlib::matrix<dlib::rgb_pixel> dlib_mat;
        dlib::assign_image(dlib_mat, image);

        // Run the face detector on the image of our action heroes, and for each face extract a
        // copy that has been normalized to 150x150 pixels in size and appropriately rotated
        // and centered.
        std::vector<dlib::matrix<dlib::rgb_pixel>> normalized_faces;
        for (const auto& rect : faces->rects)
        {
          dlib::rectangle face(rect.left, rect.top, rect.left + rect.width, rect.top + rect.height);
          const auto shape = shape_predictor_(dlib_mat, face);
          dlib::matrix<dlib::rgb_pixel> face_chip;
          dlib::extract_image_chip(dlib_mat, dlib::get_face_chip_details(shape, 150, 0.25), face_chip);
          normalized_faces.push_back(std::move(face_chip));
          cv::rectangle(frame, cv::Point(rect.left, rect.top),
                        cv::Point(rect.left + rect.width, rect.top + rect.height), cv::Scalar(0, 0, 255), 3, CV_AA);
        }

        // Calcurate face descriptors
        std::vector<dlib::matrix<float, 0, 1>> face_descriptors = net_(normalized_faces);

        // Predict name and put name on the image
        for (int i = 0; i < face_descriptors.size(); i++)
        {
          const auto& rect = faces->rects[i];
          std::string name = classifier_.predict(face_descriptors[i]);
          int text_height = 20;
          cv::Point text_pos;
          if (rect.top + rect.height + text_height > frame.rows)
            text_pos = cv::Point(rect.left, rect.top - text_height);
          else
            text_pos = cv::Point(rect.left, rect.top + rect.height + text_height);
          cv::putText(frame, name, text_pos, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255), 2, CV_AA);
        }

        // Store largest face to latest_descriptor_ (for save face service)
        auto iter = std::max_element(faces->rects.begin(), faces->rects.end(),
                                     [](const auto& l, const auto& r) {  // NOLINT(whitespace/braces)
                                       return l.width * l.height < r.width * r.height;
                                     });  // NOLINT(whitespace/braces)
        latest_descriptor_ = face_descriptors[std::distance(faces->rects.begin(), iter)];
      }
      // Publish the image.
      sensor_msgs::Image::Ptr out_image =
          cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();
      image_pub_.publish(out_image);
    }
    catch (cv::Exception& e)
    {
      NODELET_ERROR("Image processing error: %s %s %s %i", e.err.c_str(), e.func.c_str(), e.file.c_str(), e.line);
    }

    prev_stamp_ = msg->header.stamp;
  }

  /**
   * @brief Callback for save face service.
   * Save latest_descriptor_ to "[face_data_path_]/[request.name]/[0-9]{6}.dat"
   *
   * @param req Request message
   * @param res Response message
   * @return bool Succeed to save face or not.
   */
  bool saveFaceCallback(SaveFace::Request& req, SaveFace::Response& res)
  {
    if (!is_activated_)
    {
      res.success = false;
      res.message = "Face recognition is inactive. Please subscribe image topic published by face recognition first.";
      return true;
    }
    boost::mutex::scoped_lock lock(mutex_);
    auto save_path = face_data_path_ / req.name;
    // Create path
    try
    {
      if (!boost::filesystem::exists(save_path))
      {
        boost::filesystem::create_directory(save_path);
      }
    }
    catch (boost::filesystem::filesystem_error& e)
    {
      res.success = false;
      res.message = e.what();
      return true;
    }

    // Assign consecutive number.
    int max_index = -1;
    for (const auto& e : boost::make_iterator_range(boost::filesystem::directory_iterator(save_path)))
    {
      if (!boost::filesystem::is_directory(e))
        max_index = std::max(std::stoi(e.path().stem().string()), max_index);
    }
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << max_index + 1;
    const auto file_path = save_path / (ss.str() + ".dat");

    // Save face descriptor
    dlib::serialize(file_path.string()) << latest_descriptor_;

    res.success = true;
    res.message = "Saved face";
    return true;
  }

  void subscribe() final
  {
    image_sub_.subscribe(*it_, "image", 1);
    face_sub_.subscribe(*nh_, "faces", 1);
    async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(queue_size_);
    async_->connectInput(image_sub_, face_sub_);
    async_->registerCallback(boost::bind(&FaceRecognitionNodelet::faceImageCallback, this, _1, _2));
  }

  void unsubscribe() final
  {
    image_sub_.unsubscribe();
    face_sub_.unsubscribe();
    async_.reset();
  }

public:
  void onInit() final
  {
    Nodelet::onInit();
    it_ = boost::shared_ptr<image_transport::ImageTransport>(new image_transport::ImageTransport(*nh_));

    pnh_->param("queue_size", queue_size_, 3);
    pnh_->param("hz", hz_, 10);
    pnh_->param("control_hz", control_hz_, false);

    int k;
    pnh_->param("k", k, 3);
    double threshold;
    pnh_->param("threshold", threshold, 0.6);

    prev_stamp_ = ros::Time(0, 0);

    image_pub_ = advertiseImage(*pnh_, "image", 1);
    save_face_srv_ = pnh_->advertiseService("save_face", &FaceRecognitionNodelet::saveFaceCallback, this);

    std::string shape_prediction_model_path;
    pnh_->param<std::string>("shape_prediction_model_path", shape_prediction_model_path,
                             ".ros/dlib/models/shape_predictor_5_face_landmarks.dat");
    dlib::deserialize(shape_prediction_model_path) >> shape_predictor_;
    std::string recognition_model_path;
    pnh_->param<std::string>("recognition_model_path", recognition_model_path,
                             ".ros/dlib/models/dlib_face_recognition_resnet_model_v1.dat");
    dlib::deserialize(recognition_model_path) >> net_;
    std::string face_data_path;
    pnh_->param<std::string>("face_data_path", face_data_path, ".ros/dlib/face_data");
    face_data_path_ = boost::filesystem::path(face_data_path);
    classifier_ = Classifier(face_data_path_, k, threshold);
  }

  ~FaceRecognitionNodelet()
  {
    if (is_activated_)
      unsubscribe();
  }
};
}  // namespace dlib_face

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(dlib_face::FaceRecognitionNodelet, nodelet::Nodelet);
