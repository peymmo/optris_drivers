#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "libirimager/ImageBuilder.h"

class OptrisColorConvert {

  public:

    OptrisColorConvert (ros::NodeHandle n, ros::NodeHandle n_);
    ~OptrisColorConvert (void);

    void onThermalDataReceive (const sensor_msgs::ImageConstPtr& image, 
                               const sensor_msgs::CameraInfoConstPtr & cam_info);
    void onVisibleDataReceive (const sensor_msgs::ImageConstPtr& image);

    unsigned char*                    _bufferThermal;
    unsigned char*                    _bufferVisible;
    image_transport::CameraPublisher  _pubThermal;
    image_transport::Publisher        _pubVisible;
    unsigned int                      _frame;

    ros::Timer camera_timer;
    std::string _thermalimage_topic, _visibleimage_topic;
    int skip_count_thermal, cur_count_thermal;
    int skip_count_visible, cur_count_visible;

    optris::ImageBuilder              _iBuilder;
    optris::EnumOptrisColoringPalette _palette;

    ros::NodeHandle& nh_;
    ros::NodeHandle& nh_private_;
};

