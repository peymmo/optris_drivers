#include "ros/ros.h"

#include "libirimager/IRImager.h"
#include "libirimager/ImageBuilder.h"

#include "optris_colorconvert.h"

OptrisColorConvert * drv;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "optris_colorconvert_node");

  // private node handle to support command line parameters for rosrun
  ros::NodeHandle n_("~");
  ros::NodeHandle n;

  drv = new OptrisColorConvert(n, n_);
  ros::spin ();

  return 0;
}
