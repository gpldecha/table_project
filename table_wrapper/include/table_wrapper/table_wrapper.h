#ifndef TABLE_WRAPPER_H_
#define TABLE_WRAPPER_H_

#include <world_wrapper/world_wrapper.h>
#include "node/publisher.h"
#include <world_wrapper/visualisation/vis_wbox.h>
#include <optitrack_rviz/listener.h>


class Table_wrapper{

public:

    Table_wrapper(ros::NodeHandle &nh, const std::string& table_link_name, bool bVisualise,const std::string& fixed_frame="world");


    void update();

    wobj::WrapObject& get_wrapped_objects();


private:

    void initialise_table(const std::string &fixed_frame, const std::string table_link_name="table_link");

    void initialise_block(const std::string& fixed_frame, const std::string block_link_name="block_link");

private:

    ww::World_wrapper                          world_wrapper;
    std::shared_ptr<ww::Publisher>             world_publisher;

    wobj::WBox                                 wbox, wbox_block;
    bool                                       bVisualise;


};

#endif

