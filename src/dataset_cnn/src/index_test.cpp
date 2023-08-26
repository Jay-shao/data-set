#include <ros/ros.h>

/*grid_map*/
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
using namespace grid_map;
using namespace grid_map_msgs;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "index_test");
    ros::NodeHandle nh;
    grid_map::Index index;
    index(0)=1;
    index(1)=2;
    cout<<index(0)<<"||"<<index(1)<<"||"<<index.size()<<endl;

    ROS_INFO("index(0) is %d,index(1) is %d,index size is :%d",index(0), index(1),index.size());
    return 0;
}
