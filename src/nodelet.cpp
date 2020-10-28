// Copyright 2020 Tatsuro Sakaguchi<tacchan.mello.ioiq@gmail.com>

#include <string>

#include "dlib_face/nodelet.h"

namespace dlib_face
{
void Nodelet::onInit()
{
  nh_.reset(new ros::NodeHandle(getMTNodeHandle()));
  pnh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle()));
}

void Nodelet::connectionCallback(const ros::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (isSubscribed())
  {
    if (!is_activated_)
    {
      subscribe();
      is_activated_ = true;
    }
  }
  else
  {
    unsubscribe();
    is_activated_ = false;
  }
}

void Nodelet::imageConnectionCallback(const image_transport::SingleSubscriberPublisher& pub)
{
  boost::mutex::scoped_lock lock(mutex_);
  if (isSubscribed())
  {
    if (!is_activated_)
    {
      subscribe();
      is_activated_ = true;
    }
  }
  else
  {
    unsubscribe();
    is_activated_ = false;
  }
}

bool Nodelet::isSubscribed()
{
  for (const ros::Publisher& pub : publishers_)
  {
    if (pub.getNumSubscribers() > 0)
      return true;
  }
  for (const image_transport::Publisher& pub : image_publishers_)
  {
    if (pub.getNumSubscribers() > 0)
      return true;
  }
  return false;
}

image_transport::Publisher Nodelet::advertiseImage(ros::NodeHandle& nh, const std::string& topic, int queue_size)
{
  boost::mutex::scoped_lock lock(mutex_);
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&Nodelet::imageConnectionCallback, this, _1);
  image_transport::SubscriberStatusCallback disconnect_cb = boost::bind(&Nodelet::imageConnectionCallback, this, _1);
  image_transport::Publisher ret = image_transport::ImageTransport(nh).advertise(topic, 1, connect_cb, disconnect_cb);
  image_publishers_.push_back(ret);
  return ret;
}

}  // namespace dlib_face
