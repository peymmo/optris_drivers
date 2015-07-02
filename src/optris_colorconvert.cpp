
#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "libirimager/ImageBuilder.h"

#include "optris_colorconvert.h"

void OptrisColorConvert::onThermalDataReceive (const sensor_msgs::ImageConstPtr& image,
                                               const sensor_msgs::CameraInfoConstPtr & cam_info)
{
  // check for any subscribers to save computation time
  if(_pubThermal.getNumSubscribers() == 0)
     return;

  if (cur_count_thermal < skip_count_thermal) {
    cur_count_thermal++;
    return;
  } else {
    cur_count_thermal = 0;
  }

  if(_bufferThermal==NULL)
    _bufferThermal = new unsigned char[image->width * image->height * 3];

  _iBuilder.setData(image->width, image->height, (unsigned short*)&image->data[0]);
  _iBuilder.convertTemperatureToPaletteImage(_bufferThermal, true);

  sensor_msgs::ImagePtr _thermal_image(new sensor_msgs::Image);
  _thermal_image->header.stamp = image->header.stamp;
  _thermal_image->header.frame_id = "thermal_image_view";
  _thermal_image->height          = image->height;
  _thermal_image->width           = image->width;
  _thermal_image->encoding        = "rgb8";
  _thermal_image->step            = image->width*3;
  _thermal_image->data.resize(image->width * image->height * 3);
  memcpy(&_thermal_image->data[0], _bufferThermal, image->width * image->height * 3);

  _pubThermal.publish(_thermal_image);
}

void OptrisColorConvert::onVisibleDataReceive(const sensor_msgs::ImageConstPtr& image)
{
  // check for any subscribers to save computation time
  if(_pubVisible.getNumSubscribers() == 0)
     return;

  if (cur_count_visible < skip_count_visible) {
    cur_count_visible++;
    return;
  } else {
    cur_count_visible = 0;
  }

  if(_bufferVisible==NULL)
    _bufferVisible = new unsigned char[image->width * image->height * 3];

  const unsigned char* data = &image->data[0];
  _iBuilder.yuv422torgb24(data, _bufferVisible, image->width, image->height);

  sensor_msgs::ImagePtr _visible_image(new sensor_msgs::Image);

  _visible_image->header.stamp = image->header.stamp;
  _visible_image->header.frame_id = "visible_image_view";
  _visible_image->height          = image->height;
  _visible_image->width           = image->width;
  _visible_image->encoding        = "rgb8";
  _visible_image->step            = image->width * 3;
  _visible_image->data.resize(_visible_image->height * _visible_image->step);
  memcpy(&_visible_image->data[0], _bufferVisible, image->width*image->height*3);

  _pubVisible.publish(_visible_image);
}

OptrisColorConvert::OptrisColorConvert (ros::NodeHandle n, ros::NodeHandle n_):
  nh_(n),
  nh_private_(n_)
{
  _bufferThermal = NULL;
  _bufferVisible = NULL;

  int palette = 6;
  nh_private_.getParam("palette", palette);
  _palette = (optris::EnumOptrisColoringPalette) palette;

  optris::EnumOptrisPaletteScalingMethod scalingMethod = optris::eMinMax;
  int sm;
  nh_private_.getParam("paletteScaling", sm);
  if(sm>=1 && sm <=4) scalingMethod = (optris::EnumOptrisPaletteScalingMethod) sm;

  _iBuilder.setPaletteScalingMethod(scalingMethod);
  _iBuilder.setPalette(_palette);

  double tMin     = 20.;
  double tMax     = 40.;

  nh_private_.getParam("temperatureMin", tMin);
  nh_private_.getParam("temperatureMax", tMax);
  nh_private_.param("skip_count", skip_count_thermal, 3);
  skip_count_visible = skip_count_thermal;

  _iBuilder.setManualTemperatureRange((float)tMin, (float)tMax);

  it = new image_transport::ImageTransport (nh_);
  nh_private_.param<std::string>("thermal_topic", _thermalimage_topic, "thermal_image");
  nh_private_.param<std::string>("visible_topic", _visibleimage_topic, "visible_image");

  ROS_INFO ("OptrisColorConvert: subscribing to %s", _thermalimage_topic.c_str());
  subVisible = it->subscribe(_visibleimage_topic, 1, &OptrisColorConvert::onVisibleDataReceive, this);
  subThermal = it->subscribeCamera(_thermalimage_topic, 1, &OptrisColorConvert::onThermalDataReceive, this);

  _pubThermal = it->advertise("thermal_image_view", 1);
  _pubVisible = it->advertise("visible_image_view", 1);

  // set to png compression
  std::string key;
  if(ros::param::search("thermal_image/compressed/format", key))
  {
     ros::param::set(key, "png");
  }
  if(ros::param::search("thermal_image/compressed/png_level", key))
  {
     ros::param::set(key, 9);
  }
}

OptrisColorConvert::~OptrisColorConvert (void)
{
  delete it;
}


