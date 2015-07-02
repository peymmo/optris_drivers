#include "ros/ros.h"

#include "libirimager/IRImager.h"
#include "libirimager/ImageBuilder.h"

#include "optris_driver.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optris_driver_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");
  ros::NodeHandle n;

  boost::shared_ptr<OptrisDriver> drv(new OptrisDriver(n, n_));
  ros::spin ();

  return 0;
}
