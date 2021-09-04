#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>

class Depth2PointCloud1D
{
  public:
    Depth2PointCloud1D::Depth2PointCloud1D():scan_height_(20), range_min_(0.4), range_max_(6){
        subDepth = nh.subscribe("/camera/depth/image_rect_raw", 10, cbDepth, this);
        subDepthInfo = nh.subscribe("/camera/depth/camera_info", 10, cbDepthInfo, this);

    }
    
    void Depth2PointCloud1D::cbDepth(const sensor_msgs::ImageConstPtr& msg)
    {
        /* uint16 to decode */
        cv::Mat img;
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, desired_encoding='passthrough');
            img = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Could not convert from '%s' to 'uint16'.", msg->encoding.c_str());
        }

        /* Extract middle line & return smallest depth*/
        const double unit_scaling = 0.001f;
        const float constant_x = unit_scaling / fx;
        const int offset = (int)(center_y - scan_height_/2 ); //-scan_height_*2

        const uint16_t* depth_row = reinterpret_cast<const uint16_t*>(&depth_msg->data[0]);
        const int row_step = depth_msg->step / sizeof(uint16_t);
        depth_row += offset*row_step; // Offset to center of image

        pcl::PointCloud2 //pcl_conversion, pcl_ros

        for(int v = offset; v < offset+scan_height_; ++v, depth_row += row_step)
        {
            for (int u = 0; u < width; ++u) // Loop over each pixel in row
            {
                const uint16_t depth = depth_row[u];
                int index = u;

                if (std::isfinite(depth)){ // Not NaN or Inf
                    // Calculate in XYZ [m]
                    double x = (u - cx) * depth * constant_x;
                    double z = depth * 0.001f;
                }
                if(use_point(depth, ranges[index] )){
                    scan_msg->ranges[index].z = z;
                }
            }
        }
        
        /* To point cloud ros type */
    }

    void Depth2PointCloud1D::use_point(const float new_value, const float old_value) const
    {   
        const bool range_check = range_min_ <= new_value && new_value <= range_max_;
        if(!range_check)
            return false;
        
        // Finally, if they are both numerical and new_value is closer than old_value, use new_value.
        const bool shorter_check = new_value < old_value;
        return shorter_check;
    }

    void Depth2PointCloud1D::cbDepthInfo(const sensor_msgs::CameraInfoConstPtr& msg)
    {
        fx = msg->K[0];
        cx = msg->K[2];
        fy = msg->K[4];
        cy = msg->K[5];
        width = msg->width;
        height = msg->height;
    }


  private:
    ros::NodeHandle nh;
    ros::Subscriber subDepth;
    ros::Subscriber subDepthInfo;

    double fx;
    double fy;
    double cx;
    double cy;
    int width;
    int height;
    int offset;
    int scan_height_;
    double range_min_;
    double range_max_;
    double *ranges;

};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_2_1dpc");
    ros::NodeHandle nh;
    ros::spin();
    
    return 0;
}