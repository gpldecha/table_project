#include "table_broadcaster/table_broadcaster.h"
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace tab{

Table_broadcaster::Table_broadcaster(ros::NodeHandle& node,
                                     const std::array<float,3>& origin,
                                     const std::array<float,4>& orientation, const std::array<float,3> &offset,
                                     std::string save_path)
{
    // set services

    s_transform = node.advertiseService("set_table_tf",&Table_broadcaster::service_transform_callback,this);
    s_cmd       = node.advertiseService("table_cmd",&Table_broadcaster::service_str_callback,this);
    b_got_tf = false;
    services["set_location"] = SET_LOCATION;
    services["save"]         = SAVE;

    this->offset = offset;


    tf::Matrix3x3 rotation;
    rotation.setRPY(0,0,M_PI);
    tf::Quaternion q;
    rotation.getRotation(q);
    std::cout<< "Rotation : (" << q.getX() << "," << q.getY() << "," << q.getZ() << "," << q.getW() << ") " << std::endl;

     std::cout<< "origin: (" << origin[0] << "," << origin[1] << "," << origin[2] << ")" << std::endl;
     std::cout<< "orientation: (" << orientation[0] << "," << orientation[1] << "," << orientation[2] << "," << orientation[3] << ")" << std::endl;

    // initial position


    urdf_message.transform.translation.x = origin[0];
    urdf_message.transform.translation.y = origin[1];
    urdf_message.transform.translation.z = origin[2];
    urdf_message.transform.rotation.x = orientation[0];
    urdf_message.transform.rotation.y = orientation[1];
    urdf_message.transform.rotation.z = orientation[2];
    urdf_message.transform.rotation.w = orientation[3];

    // path to save transforms

   // boost::filesystem::path full_path( boost::filesystem::current_path() );

    path_to_save = save_path;
    std::cout<< "path_to_save: " << path_to_save << std::endl;

}

void Table_broadcaster::set_frames(const std::string &fixed_frame_,
                                   const std::string &target_frame_vision_,
                                   const std::string& target_frame_rviz_)
{
    fixed_frame         = fixed_frame_;
    target_frame_vision = target_frame_vision_;
    target_frame_rviz   = target_frame_rviz_;


    urdf_message.child_frame_id  = target_frame_rviz;
    urdf_message.header.frame_id = fixed_frame;

}


void Table_broadcaster::update(){

        urdf_message.header.stamp = ros::Time::now();
        broadcaster.sendTransform(urdf_message);

}



bool Table_broadcaster::service_transform_callback(table_broadcaster::Transform_cmd::Request& req,
                                     table_broadcaster::Transform_cmd::Response& resp)
{
         urdf_message.transform = req.transform;
         resp.str = "transform recieved";
         return true;
}

bool Table_broadcaster::service_str_callback(table_broadcaster::String_cmd::Request& req,
                                             table_broadcaster::String_cmd::Response& resp){
    service srv;
    try {
        srv = services.at(req.str);
    }
    catch (const std::out_of_range& e) {
        resp.str = "ERROR no such service exists: " + req.str + " !!!";
        return false;
    }


    switch(srv){
    case SET_LOCATION:
    {
        set_location_table();
        resp.str = "set_location service";
        break;
    }
    case SAVE:
    {
        if(save()){
            resp.str = "save service sucessful";
        }else{
            return false;
        }
        break;
    }

    }



    return true;

}

void Table_broadcaster::set_location_table(){
    while(!b_got_tf){
        try{
            tf_listener.lookupTransform(fixed_frame,target_frame_vision, ros::Time(0), tf_transform);

           // vis_to_rviz.vision_to_rviz(tf_transform);

            urdf_message.transform.translation.x = tf_transform.getOrigin().x() + offset[0];
            urdf_message.transform.translation.y = tf_transform.getOrigin().y() + offset[1];
            urdf_message.transform.translation.z = tf_transform.getOrigin().z() + offset[2];

         /*   urdf_message.transform.rotation.x = tf_transform.getRotation().getX();
            urdf_message.transform.rotation.y = tf_transform.getRotation().getY();
            urdf_message.transform.rotation.z = tf_transform.getRotation().getZ();
            urdf_message.transform.rotation.w = tf_transform.getRotation().getW();*/


            b_got_tf = true;
        }
        catch (tf::TransformException ex){
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    b_got_tf=false;

}

bool Table_broadcaster::save(){

    std::string folder_save = path_to_save + "/" + target_frame_rviz;

    if( !(boost::filesystem::exists(folder_save))){
        if (boost::filesystem::create_directory(folder_save)){
            std::cout << "new folder " + target_frame_rviz + " created" << "\n";
        }else{
            std::cerr << "FAILED to create new folder " + target_frame_rviz << "\n";
            return false;
        }
    }

    std::string file_origin      = folder_save  + "/origin.csv";
    std::string file_orientation = folder_save  + "/orientation.csv";

    std::ofstream file_stream1(file_origin);
    if(file_stream1.is_open()){

        file_stream1 << urdf_message.transform.translation.x << ","
                     << urdf_message.transform.translation.y << ","
                     << urdf_message.transform.translation.z << std::endl;

        file_stream1.close();
        std::cout<< "saved " + file_origin + " !" << std::endl;
    }else{
        std::cerr << " FAILED to open: " + file_origin << std::endl;
        return false;
    }

    std::ofstream file_stream2(file_orientation);
    if(file_stream2.is_open()){

        file_stream2 << urdf_message.transform.rotation.x << ","
                    << urdf_message.transform.rotation.y << ","
                    << urdf_message.transform.rotation.z << ","
                    << urdf_message.transform.rotation.w << std::endl;

        file_stream2.close();
        std::cout<< "saved " + file_orientation + " !" << std::endl;
    }else{
        std::cerr << " FAILED to open: " + file_orientation << std::endl;
        return false;
    }

    return true;
}

bool Table_broadcaster::load(std::array<float,3>& origin,std::array<float,4>& orientation,
                                    std::string path_to_save,std::string target_frame_rviz){

    std::string folder_load = path_to_save + "/" + target_frame_rviz;

    if( !(boost::filesystem::exists(folder_load))){
        std::cerr << "FAILED to open folder: " + target_frame_rviz << "\n";
        return false;
    }


    std::string file_origin      = folder_load  + "/origin.csv";
    std::string file_orientation = folder_load  + "/orientation.csv";

    std::ifstream file_stream_origin(file_origin);
    if(file_stream_origin.is_open()){
        std::string line;
        while(std::getline(file_stream_origin, line)){
            std::istringstream s(line);
            std::string field;
            std::size_t i = 0;
            while (getline(s, field,',')){
                origin[i] = boost::lexical_cast<float>(field);
                i++;
            }
        }
    }else{
        std::cerr<< "failed to open: " << file_origin << std::endl;
        return false;
    }

    std::ifstream file_stream_orientation(file_orientation);
    if(file_stream_origin.is_open()){
        std::string line;
        while(std::getline(file_stream_orientation, line)){
            std::istringstream s(line);
            std::string field;
            std::size_t i = 0;
            while (getline(s, field,',')){
                orientation[i] = boost::lexical_cast<float>(field);
                i++;
            }
        }
    }else{
        std::cerr<< "failed to open: " << file_origin << std::endl;
        return false;
    }

    return true;
}

}
