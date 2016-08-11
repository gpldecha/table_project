#include  "table_wrapper/table_wrapper.h"
#include <cmath>

Table_wrapper::Table_wrapper(ros::NodeHandle &nh,const std::string& table_link_name, bool bVisualise,const std::string& fixed_frame):
bVisualise(bVisualise)
{


    // get table box

    initialise_table(fixed_frame,table_link_name);
    initialise_block(fixed_frame,"/block_link");

    if(bVisualise){

        world_publisher = std::shared_ptr<ww::Publisher>(new ww::Publisher("table_block/visualization_marker",&nh,&world_wrapper));
        world_publisher->init(fixed_frame);
        world_publisher->update_position();
    }

}


void Table_wrapper::initialise_table(const std::string& fixed_frame,const std::string table_link_name){


    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,table_link_name,transform);

    tf::Vector3  wall_origin = transform.getOrigin();

    geo::fCVec3 origin       = {{(float)wall_origin.x(),(float)wall_origin.y(),(float)wall_origin.z()}};//{{0,0,-0.02/2}};
    float      dz            = 0.05;
    origin(2)                = origin(2)-dz/2;
    geo::fCVec3 dim          = {{0.7,0.5,0.05}};
    geo::fCVec3 orientation  = {{0,0,0}};
    wbox = wobj::WBox(table_link_name,dim,origin,orientation);
    world_wrapper.wrapped_objects.push_back_box(&wbox);
}

void Table_wrapper::initialise_block(const std::string& fixed_frame, const std::string block_link_name){

    tf::StampedTransform transform;
    opti_rviz::Listener::get_tf_once(fixed_frame,block_link_name,transform);

    tf::Vector3  block_origin = transform.getOrigin();

    geo::fCVec3 origin       = {{(float)block_origin.x(),(float)block_origin.y(),(float)block_origin.z()}};//{{0,0,-0.02/2}};
    geo::fCVec3 dim          = {{0.04,0.02,0.02}};
    geo::fCVec3 orientation  = {{0,0,0}};
    wbox_block = wobj::WBox(block_link_name,dim,origin,orientation);
    world_wrapper.wrapped_objects.push_back_box(&wbox_block);

}

wobj::WrapObject& Table_wrapper::get_wrapped_objects(){
    return world_wrapper.wrapped_objects;
}


void Table_wrapper::update(){

    world_publisher->update_position();


    if(bVisualise){


        /*opti_rviz::type_conv::vec2tf(peg_sensor_model->get_closet_point(SURFACE),v_surf[0]);
        opti_rviz::type_conv::vec2tf(peg_sensor_model->get_closet_point(EDGE),v_edge[0]);

        vis_proj_sur->update(v_surf);
        vis_proj_sur->publish();

        vis_proj_edge->update(v_edge);
        vis_proj_edge->publish();

        vis_points->publish();
        vis_socket->publish();*/

        world_publisher->publish();
    }
}

