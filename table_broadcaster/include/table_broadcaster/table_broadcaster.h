#ifndef TABLE_BROADCASTER_H_
#define TABLE_BROADCASTER_H_

// STL

#include <map>
#include <array>

// Ros

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// table_rviz_control

#include <table_broadcaster/Transform_cmd.h>
#include <table_broadcaster/String_cmd.h>

namespace tab{

typedef enum{SET_LOCATION,SAVE} service;

class Table_broadcaster{


public:

    Table_broadcaster(ros::NodeHandle& node,const std::array<float,3> &origin,
                      const std::array<float,4> &orientation,const std::array<float,3> &offset,
                      std::string save_path);

    void set_frames(const std::string &fixed_frame_, const std::string &target_frame_vision_, const std::string& target_frame_rviz_);

    void update();

    void set_location_table();


private:

    bool save();

public:

    static bool load(std::array<float,3>& origin,std::array<float,4>& orientation,
                     std::string path_to_save,std::string target_frame_rviz);

private:

    bool service_transform_callback(table_broadcaster::Transform_cmd::Request& req,table_broadcaster::Transform_cmd::Response& resp);

    bool service_str_callback(table_broadcaster::String_cmd::Request &req, table_broadcaster::String_cmd::Response& resp);


private:

    // service
    ros::ServiceServer                  s_transform, s_cmd;
    std::map<std::string,tab::service>  services;

    // broadcast
    tf::TransformBroadcaster            broadcaster;
    geometry_msgs::TransformStamped     urdf_message;

    // tf listener
    tf::TransformListener               tf_listener;
    tf::StampedTransform                tf_transform;
    bool b_got_tf;
    bool b_load;
  //  v2r vis_to_rviz;
    std::array<float,3> offset;

    std::string fixed_frame, target_frame_vision, target_frame_rviz;
    std::string path_to_save;




};

}

#endif

