// STL

#include <map>
#include <string>

// Boost

#include <boost/lexical_cast.hpp>

// catkin package

#include "optitrack_rviz/listener.h"
#include "optitrack_rviz/broadcaster.h"

// table_rviz_control

#include <table_broadcaster/table_broadcaster.h>


int find_index(int argc,const std::vector<std::string>& argv,std::string str){
    for(int i = 0; i < argc;i++){
        if(argv[i] == str){
            return i;
        }
    }
    return -1;
}

bool process_input(int argc, char **argv,std::map<std::string,std::string>& input,
                   std::array<float,3>& origin,std::array<float,4>& orientation,std::array<float,3>& offset){

    if(argc < static_cast<int>(input.size())){
        std::string error_msg = "options ";
        for(auto it = input.begin(); it != input.end();it++){
            error_msg += it->first + " ";
        }
        error_msg += "not defined!";
        ROS_ERROR("%s",error_msg.c_str());
        return false;
    }else{

        int index = 0;
        std::string empty = "";

        std::vector<std::string> input_args(argc);
        for(int i = 0; i < argc;i++){
            input_args[i] = std::string(argv[i]);
        }


        for(auto it = input.begin(); it != input.end();it++){
                index = find_index(argc,input_args,it->first);

                if(index == -1 && (it->second == empty)){
                    ROS_ERROR("%s [arg] not specified",it->first.c_str());
                    return false;
                }else if(index != -1){
                    if(it->first == "-origin"){
  //                      std::cout<< "origin" << std::endl;
//                        std::cout<< "index+1: " << index + 1 << std::endl;
                        origin[0] = boost::lexical_cast<float>(input_args[index+1]);
                        origin[1] = boost::lexical_cast<float>(input_args[index+2]);
                        origin[2] = boost::lexical_cast<float>(input_args[index+3]);

                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];
                    }else if(it->first == "-orientation"){
                       // std::cout<< "orientation" << std::endl;
                        //std::cout<< "index+1: " << index + 1 << std::endl;
                        orientation[0] = boost::lexical_cast<float>(input_args[index+1]);
                        orientation[1] = boost::lexical_cast<float>(input_args[index+2]);
                        orientation[2] = boost::lexical_cast<float>(input_args[index+3]);
                        orientation[3] = boost::lexical_cast<float>(input_args[index+4]);
                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3] + " " + input_args[index+4];
                    }else if(it->first == "-offset"){
                        offset[0] = boost::lexical_cast<float>(input_args[index+1]);
                        offset[1] = boost::lexical_cast<float>(input_args[index+2]);
                        offset[2] = boost::lexical_cast<float>(input_args[index+3]);
                        (it->second) = input_args[index+1] + " " + input_args[index+2] + " " + input_args[index+3];

                    }else{
                        (it->second) =  std::string(argv[index + 1]);
                    }
                }
        }

        return true;
    }
}

void print_input_options(const std::map<std::string,std::string>& input){
    for(auto it = input.begin(); it != input.end();it++){
        ROS_INFO("%s\t%s",it->first.c_str(),it->second.c_str());
    }
}

std::string num2str(float num){
    return boost::lexical_cast<std::string>(num);
}

int main(int argc,char** argv)
{

    std::map<std::string,std::string> input;

    input["-fixed_frame"]          = "";
    input["-target_frame_vision"]  = "";
    input["-target_frame_rviz"]    = "";
    input["-origin"]               = "0 0 0";
    input["-orientation"]          = "0 0 0 1";
    input["-offset"]               = "0 0 0";
    input["-save"]                 = "/home/guillaume/";
    input["-load"]                 = "False";
    input["-rate"]                 = "100";

    std::array<float,3> origin,offset;
    std::array<float,4> orientation;
    offset = {{0,0,0}};


    if(!process_input(argc,argv,input,origin,orientation,offset)){
        return -1;
    }


    if(input["-load"] == "True"){
        std::cout<< "== load == " << std::endl;
        tab::Table_broadcaster::load(origin,orientation,input["-save"],input["-target_frame_rviz"]);
        input.at("-origin") = num2str(origin[0]) + " " + num2str(origin[1]) + " " + num2str(origin[2]);
        input.at("-orientation") = num2str(orientation[0]) + " " + num2str(orientation[1]) + " " + num2str(orientation[2]) + " "
                + num2str(orientation[3]);
        input.at("-offset") = num2str(offset[0]) + " " + num2str(offset[1]) + " " + num2str(offset[2]);
    }


    print_input_options(input);

    ros::init(argc, argv,"table_broadcaster",ros::init_options::AnonymousName);
    ros::NodeHandle node;
    tab::Table_broadcaster table_broadcaster(node,origin,orientation,offset,input["-save"]);
  //  table_broadcaster.set_frames(input["-fixed_frame"],input["-target_frame_vision"],input["-target_frame_rviz"]);


    opti_rviz::Listener     listener(input["-fixed_frame"],input["-target_frame_vision"]);
    opti_rviz::Broadcaster  broadcaster(input["-fixed_frame"],input["-target_frame_rviz"]);
    tf::Quaternion q;
    q.setValue(orientation[0],orientation[1],orientation[2],orientation[3]);
    tf::Vector3     origin_table;
    tf::Matrix3x3   orientation_table;
    orientation_table.setRotation(q);


    int Hz = boost::lexical_cast<int>(input["-rate"]);
    ros::Rate rate(Hz);
    while(node.ok()){

        //table_broadcaster.update();

        listener.update(origin_table,orientation_table);

       // orientation_table.getRotation(q);

        broadcaster.update(origin_table,orientation_table);


        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
