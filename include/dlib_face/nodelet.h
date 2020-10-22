#ifndef DLIB_FACE_NODELET_H_
#define DLIB_FACE_NODELET_H_

#include <string>
#include <vector>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <boost/thread.hpp>
#include <image_transport/image_transport.h>

namespace dlib_face
{
/**
 * @brief The class is a nodelet that automatically pauses when not subscribed.
 *
 */
class Nodelet : public nodelet::Nodelet
{
public:
  Nodelet() : is_activated_(false)
  {
  }

protected:
  /**
   * @brief Nodelet initializer
   *
   */
  void onInit() override;

  /**
   * @brief Callback called when the topic published by the publisher is subscribed/unsubscribed.
   *
   * @param publisher Topic publisher
   */
  virtual void connectionCallback(const ros::SingleSubscriberPublisher& publisher);
  /**
   * @brief Callback called when the image topic published by the publisher is subscribed/unsubscribed.
   *
   * @param publisher Image topic publisher
   */
  virtual void imageConnectionCallback(const image_transport::SingleSubscriberPublisher& publisher);
  /**
   * @brief Is anyone subscribe topics published by myself.
   *
   * @return bool Subscribe or not
   */
  virtual bool isSubscribed();

  /**
   * @brief Subscribe some topics
   *
   */
  virtual void subscribe() = 0;
  /**
   * @brief Unsubscribe some topics
   *
   */
  virtual void unsubscribe() = 0;

  /**
   * @brief Advertise the topic and register the publisher
   *
   * @tparam T              Message type
   * @param nh              Node handle
   * @param topic           Publish topic name
   * @param queue_size      Queue size of publisher
   * @return ros::Publisher Registered publisher
   */
  template <class T>
  ros::Publisher advertise(ros::NodeHandle& nh, std::string topic, int queue_size)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ros::SubscriberStatusCallback connect_cb = boost::bind(&Nodelet::connectionCallback, this, _1);
    ros::SubscriberStatusCallback disconnect_cb = boost::bind(&Nodelet::connectionCallback, this, _1);
    ros::Publisher ret = nh.advertise<T>(topic, queue_size, connect_cb, disconnect_cb);
    publishers_.push_back(ret);

    return ret;
  };
  /**
   * @brief Advertise the image topic and register the publisher
   *
   * @param nh              Node handle
   * @param topic           Publish image topic name
   * @param queue_size      Queue size of publisher
   * @return ros::Publisher Registered publisher
   */
  image_transport::Publisher advertiseImage(ros::NodeHandle& nh, const std::string& topic, int queue_size);

  boost::mutex mutex_;

  std::vector<ros::Publisher> publishers_;
  std::vector<image_transport::Publisher> image_publishers_;

  boost::shared_ptr<ros::NodeHandle> nh_;
  boost::shared_ptr<ros::NodeHandle> pnh_;

  bool is_activated_;

private:
};
}  // namespace dlib_face

#endif  // DLIB_FACE_NODELET_H_
