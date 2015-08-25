
#include <ros/ros.h>
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

#include <sys/stat.h>

#include "filter.h"
#include "simple_stat.h"

using namespace std;

class OptrisDriver {

  public:

    static void onThermalFrame(unsigned short* image, unsigned int w, unsigned int h, long long timestamp, void *arg);
    static void onVisibleFrame(unsigned char* image, unsigned int w, unsigned int h, long long timestamp, void *arg);

    OptrisDriver (ros::NodeHandle n, ros::NodeHandle n_);
    ~OptrisDriver (void);

    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_private_;

    camera_info_manager::CameraInfoManager *cinfo_manager_;

    image_transport::ImageTransport *it;
    image_transport::CameraPublisher _thermal_pub;
    image_transport::Publisher _visible_pub;
    boost::shared_ptr < dynamic_reconfigure::Server <optris_drivers::RadparmsConfig> > server;
    dynamic_reconfigure::Server<optris_drivers::RadparmsConfig>::CallbackType f;

    ros::Publisher _timer_pub;
    ros::Publisher _temp_pub;

    ros::ServiceServer sAuto;
    ros::ServiceServer sForce;

    unsigned int _img_cnt;
    optris::IRImager * _imager;
    unsigned char * bufferRaw;
    bool streaming_ok;
    bool processing_image;
    ros::Timer camera_timer;
    Filter * optris_timer_filter;
    double prev_timestamp;
    SimpleStat cb_duration;

    bool use_device_timer;
    int filter_size;
    std::string _thermalframe_id,_visibleframe_id;
    std::string _camera_calibration_url;
    std::string _node_name;
    double emmisivity;
    double transmissivity;
    double sampling_multiplier;

    bool onAutoFlag(optris_drivers::AutoFlag::Request &req, optris_drivers::AutoFlag::Response &res);
    bool onForceFlag(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void camera_timer_callback (const ros::TimerEvent& e);
    void dyn_reconfig_cb(optris_drivers::RadparmsConfig &config, uint32_t level);
};


