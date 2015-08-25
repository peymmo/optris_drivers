#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/TimeReference.h>
#include "std_msgs/Float32.h"
#include "std_srvs/Empty.h"
#include "optris_drivers/AutoFlag.h"
#include <optris_drivers/Temperature.h>

#include <dynamic_reconfigure/server.h>
#include <optris_drivers/RadparmsConfig.h>

#include "libirimager/IRImager.h"
#include "libirimager/ImageBuilder.h"

#include "optris_driver.h"

#include <sys/stat.h>
using namespace std;

/**
 * Callback method from image processing library (called at configured frame rate in xml file)
 * @param[in] image thermal image in unsigned short format, i.e., float temperature = ((float)image[i] -1000.f)/10.f)
 * @param[in] w image width
 * @param[in] h image height
 */
void OptrisDriver::onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, long long timestamp, void *arg)
{
  ros::Time ros_now = ros::Time::now();
  OptrisDriver *p = (OptrisDriver *) arg;
  ros::Time ts;

  if (!p->use_device_timer)
  {
    ts = ros_now;
    ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-p->prev_timestamp)*1000.0));
    p->prev_timestamp = ros_now.toSec();
  } else {
    uint64_t device_time = timestamp*100;

    double device_time_in_sec = static_cast<double>(device_time)/10000000.0;
    double ros_time_in_sec = ros_now.toSec();

    double time_diff = ros_time_in_sec-device_time_in_sec;

    p->optris_timer_filter->addSample(time_diff);

    double filtered_time_diff = p->optris_timer_filter->getMedian();

    double corrected_timestamp = device_time_in_sec+filtered_time_diff;

    ts.fromSec(corrected_timestamp);

    ROS_DEBUG("Time interval between frames: %.4f ms", (float)((corrected_timestamp-p->prev_timestamp)*1000.0));

    p->prev_timestamp = corrected_timestamp;
  }

  /* thermal image */
  sensor_msgs::ImagePtr _thermal_image(new sensor_msgs::Image);
  _thermal_image->header.seq = ++p->_img_cnt;
  _thermal_image->header.stamp = ts;
  _thermal_image->header.frame_id = p->_thermalframe_id;
  _thermal_image->height          = p->_imager->getHeight();
  _thermal_image->width           = p->_imager->getWidth();
  _thermal_image->encoding        = "mono16";
  _thermal_image->step            = _thermal_image->width * 2;
  _thermal_image->data.resize(_thermal_image->height * _thermal_image->step);
  memcpy(&_thermal_image->data[0], image, w * h * sizeof(*image));
  /* thermal camera info */
  sensor_msgs::CameraInfoPtr _thermal_camera_info(new sensor_msgs::CameraInfo);
  _thermal_camera_info = boost::make_shared<sensor_msgs::CameraInfo>(p->cinfo_manager_->getCameraInfo());
  _thermal_camera_info->header.stamp=_thermal_image->header.stamp;
  _thermal_camera_info->header.frame_id = _thermal_image->header.frame_id;
  _thermal_camera_info->height          = _thermal_image->height;
  _thermal_camera_info->width           = _thermal_image->width;
  /* publish thermal image */
  p->_thermal_pub.publish(_thermal_image, _thermal_camera_info);

  sensor_msgs::TimeReferencePtr _optris_timer(new sensor_msgs::TimeReference);
  _optris_timer->header.frame_id=_thermal_image->header.frame_id;
  _optris_timer->header.seq = _thermal_image->header.seq;
  _optris_timer->header.stamp = _thermal_image->header.stamp;
  _optris_timer->time_ref.fromNSec(timestamp*100);
  p->_timer_pub.publish(_optris_timer);

  optris_drivers::TemperaturePtr _internal_temperature(new optris_drivers::Temperature);
  _internal_temperature->header.frame_id=_thermal_image->header.frame_id;
  _internal_temperature->header.seq=_thermal_image->header.seq;
  _internal_temperature->header.stamp = _thermal_image->header.stamp;
  _internal_temperature->temperature_flag = p->_imager->getTempFlag();
  _internal_temperature->temperature_box = p->_imager->getTempBox();
  _internal_temperature->temperature_chip = p->_imager->getTempChip();
  p->_temp_pub.publish(_internal_temperature);

  p->cb_duration.add_data (ros::Time::now().toSec() - ros_now.toSec());
}

void OptrisDriver::onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, long long timestamp, void *arg)
{
  OptrisDriver *p = (OptrisDriver *) arg;

  if(p->_visible_pub.getNumSubscribers()==0) return;

  sensor_msgs::ImagePtr _visible_image(new sensor_msgs::Image);

  _visible_image->header.seq   = p->_img_cnt;
  _visible_image->header.stamp = ros::Time::now();
  _visible_image->header.frame_id = p->_visibleframe_id;
  _visible_image->height          = p->_imager->getVisibleHeight();
  _visible_image->width           = p->_imager->getVisibleWidth();
  _visible_image->encoding        = "yuv422";
  _visible_image->step            = _visible_image->width * 2;
  _visible_image->data.resize(_visible_image->height * _visible_image->step);
  memcpy(&_visible_image->data[0], image, 2 * w * h * sizeof(*image));

  p->_visible_pub.publish(_visible_image);
}

bool OptrisDriver::onAutoFlag(optris_drivers::AutoFlag::Request &req, optris_drivers::AutoFlag::Response &res)
{
  _imager->setAutoFlag(req.autoFlag);
  return true;
}

bool OptrisDriver::onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
  _imager->forceFlagEvent();
  return true;
}

OptrisDriver::OptrisDriver(ros::NodeHandle n, ros::NodeHandle n_):
  nh_(n),
  nh_private_(n_)
{
  // Header frame_id should be optical frame of camera
  nh_private_.param<std::string>("thermal_frame_id", _thermalframe_id, "thermal_image_optical_frame");
  nh_private_.param<std::string>("visible_frame_id", _visibleframe_id, "visible_image_optical_frame");
  nh_private_.param<bool>("use_device_timer", use_device_timer, true);
  nh_private_.param<int>("filter_size", filter_size, 100);
  nh_private_.getParam("camera_name", _node_name);
  nh_private_.param<double>("emmisivity", emmisivity, 100.0);
  nh_private_.param<double>("transmissivity", transmissivity, 0.0);
  nh_private_.param<double>("sampling_multiplier", sampling_multiplier, 10.0);

  streaming_ok = false;
  processing_image = false;

  // filter
  if (use_device_timer)
    ROS_INFO("optris_driver: using device timer, filter size = %d", filter_size);
  optris_timer_filter = new Filter(filter_size);

  // set up information manager
   if (!nh_private_.getParam("calibration_file", _camera_calibration_url))
   {
     _camera_calibration_url = "";
   }

   cinfo_manager_ = new camera_info_manager::CameraInfoManager(nh_private_, _node_name, _camera_calibration_url);
   if (cinfo_manager_->isCalibrated())
   {
       ROS_INFO("camera has loaded calibration file");
   }

  std::string xmlConfig = "";
  nh_private_.getParam("xmlConfig", xmlConfig);

  // A specific configuration file for each imager device is needed (cf. config directory)
  struct stat s;
  if(stat(xmlConfig.c_str(), &s) != 0)
  {
    ROS_ERROR ("Could not find xml config file: %s", xmlConfig.c_str());
    exit (1);
  } else {
    ROS_INFO ("Initializing camera with xml config file: %s", xmlConfig.c_str());
  }

  _imager = new optris::IRImager(xmlConfig.c_str());
  _imager->setRadiationParameters((float)emmisivity, (float)transmissivity);
  bufferRaw = new unsigned char[_imager->getRawBufferSize()];
  _imager->setFrameCallback(onThermalFrame);
  _imager->setVisibleFrameCallback(onVisibleFrame);

  it = new image_transport::ImageTransport (nh_);

  //image_transport::Publisher tpub = it.advertise("thermal_image", 1);
  _thermal_pub = it->advertiseCamera("image_raw", 1);

  if(_imager->hasBispectralTechnology())
  {
    _visible_pub = it->advertise("visible_image", 1);
  }

  // advertise the camera internal timer
   _timer_pub= nh_.advertise<sensor_msgs::TimeReference>("optris_timer", 1 );

  sAuto  = nh_private_.advertiseService("auto_flag",  &OptrisDriver::onAutoFlag, this);
  sForce = nh_private_.advertiseService("force_flag", &OptrisDriver::onForceFlag, this);

  //advertise all the camera Temperature in a single custom message
  _temp_pub = nh_.advertise <optris_drivers::Temperature> ("internal_temperature", 1);

  //setup dynamic reconfigure server  
  server = boost::make_shared < dynamic_reconfigure::Server <optris_drivers::RadparmsConfig> > (nh_private_);
  f = boost::bind(&OptrisDriver::dyn_reconfig_cb, this, _1, _2);
  server->setCallback(f);

  ros::Duration timer_delay(1.0/(_imager->getMaxFramerate()*sampling_multiplier));
  ROS_INFO ("OptrisDriver: camera timer duration = %f", timer_delay.toSec());
  camera_timer = nh_.createTimer (timer_delay, &OptrisDriver::camera_timer_callback, this);
  ROS_INFO("OptrisDriver: init done");
}

OptrisDriver::~OptrisDriver(void)
{
  fprintf (stderr, "optris_driver: stats %d, %f, %f\n", cb_duration.n, cb_duration.mean,
           cb_duration.std_deviation);
  fprintf (stderr, "optris_driver: stats %f, %f\n", cb_duration.min_x, cb_duration.max_x);
  delete it;
  delete cinfo_manager_;
  delete _imager;
  delete bufferRaw;
}

void OptrisDriver::camera_timer_callback (const ros::TimerEvent& e)
{
  if (! processing_image) {
    processing_image = true;
    if (streaming_ok) {
      if (_imager->getFrame(bufferRaw)) {
        _imager->process(bufferRaw, this);
        _imager->releaseFrame();
      }
    }
    processing_image = false;
  }
}

void OptrisDriver::dyn_reconfig_cb(optris_drivers::RadparmsConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %f %f", config.emmisivity, config.transmissivity);
  streaming_ok = false;
  _imager->stopStreaming();
  emmisivity = config.emmisivity;
  transmissivity = config.transmissivity;
  _imager->setRadiationParameters((float)emmisivity, (float)transmissivity);
  streaming_ok = _imager->startStreaming();
}



