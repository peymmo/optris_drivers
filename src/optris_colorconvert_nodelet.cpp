#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "optris_colorconvert.h"

namespace optris_drivers
{

class OptrisColorConvertNodelet : public nodelet::Nodelet
{

public:

  OptrisColorConvertNodelet()  {}

  ~OptrisColorConvertNodelet() {}

private:
  virtual void onInit()
  {
    drv.reset(new OptrisColorConvert(getNodeHandle(), getPrivateNodeHandle()));
  };

  boost::shared_ptr<OptrisColorConvert> drv;
};

PLUGINLIB_DECLARE_CLASS(optris_drivers, OptrisColorConvertNodelet, optris_drivers::OptrisColorConvertNodelet, nodelet::Nodelet);
//PLUGINLIB_DECLARE_CLASS(nodelet_tutorial_math, Plus, nodelet_tutorial_math::Plus, nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(optris_drivers::OptrisDriverNodelet, nodelet::Nodelet);

}


