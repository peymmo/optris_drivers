#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include "optris_driver.h"

namespace optris_drivers
{

class OptrisDriverNodelet : public nodelet::Nodelet
{

public:

  OptrisDriverNodelet()  {}

  ~OptrisDriverNodelet() {}

private:
  virtual void onInit()
  {
    drv.reset(new OptrisDriver(getNodeHandle(), getPrivateNodeHandle()));
    drv->run();
  };

  boost::shared_ptr<OptrisDriver> drv;
};

PLUGINLIB_DECLARE_CLASS(optris_drivers, OptrisDriverNodelet, optris_drivers::OptrisDriverNodelet, nodelet::Nodelet);
//PLUGINLIB_DECLARE_CLASS(nodelet_tutorial_math, Plus, nodelet_tutorial_math::Plus, nodelet::Nodelet);
//PLUGINLIB_EXPORT_CLASS(optris_drivers::OptrisDriverNodelet, nodelet::Nodelet);

}


