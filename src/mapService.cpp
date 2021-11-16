#include "ros/ros.h"

// #include <hector_mapping/ResetMapping.h>
#include <nav_msgs/GetMap.h>
#include <iostream>
#include <fstream>

int main(int argc, char **argv){
    ros::init(argc, argv, "mapService_client_");
    ros::NodeHandle n;
    std::string file_path_;
    n.getParam("file_path", file_path_);

    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("mapCallback");
    nav_msgs::GetMap srv_map;
    // srv_map.request.initial_pose = resetPose;

    if (client.call(srv_map))
    {
      ROS_INFO("mapService_client_Node: Map service called successfully");
      const nav_msgs::OccupancyGrid& map(srv_map.response.map);

      std::ofstream newFile;
      newFile.open(file_path_.c_str(), std::ios::out | std::ios::trunc);
      if(!newFile)     
        std::cout << "Can't open file!\n";
      else
        std::cout<<"File open successfully!\n";
      int size = map.info.width * map.info.height;
      std::cout<<"size: " << size ;
      for (int i=0; i<size; i++)
      {
        newFile << map.data[i] << ',';
      
        if (i%512==0)
        {
          newFile << '\n';
        }
      }
    }
    else
    {
        ROS_ERROR("Failed to save occu map...");
    }
    return 0;
}