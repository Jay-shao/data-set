#include <ros/ros.h>
#include <iostream>
#include <vector>

/*grid_map*/
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/ProcessFile.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>

//image
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>

#include "grid_map_msgs/MyGridMap.h"

using namespace grid_map;
using namespace grid_map_msgs;
using namespace std;

class GridMapToImage
{
private:

    ros::NodeHandle nh;
    ros::Publisher gridPub_;
    ros::ServiceServer gridMapServer_;

public:
    GridMapToImage();
    bool doGridMapToImage(grid_map_msgs::MyGridMapRequest& req, grid_map_msgs::MyGridMapResponse& resp);


};

GridMapToImage::GridMapToImage()
{
    gridPub_ = nh.advertise<grid_map_msgs::GridMap>("/cnn_data_set/local_height_map", 10, true);
    gridMapServer_ = nh.advertiseService("/gridmap_to_image", &GridMapToImage::doGridMapToImage, this);
}

bool GridMapToImage::doGridMapToImage(grid_map_msgs::MyGridMapRequest& req, grid_map_msgs::MyGridMapResponse& resp)
{
    grid_map::GridMap map_;
    grid_map::GridMapRosConverter::fromMessage(req.map, map_);
    grid_map::Matrix& elevationData = map_["elevation"];
    float heightData,mapData;
    
    vector<vector<float>> orinElevationSubMap;
    vector<float> tmp;
    Index mapStartIndex(req.index_1 - req.window, req.index_2 - req.window);
    Index mapBufferSize(req.window, req.window);
    ros::Rate rate(30.0);
    ros::Time time = ros::Time::now();

    for (SubmapIterator it(map_, mapStartIndex, mapBufferSize); !it.isPastEnd(); ++it)
    {
        const grid_map::Index index(*it);
        heightData = elevationData(index(0), index(1));
        tmp.push_back(heightData);
    }

    for (int i = 0; i < req.window; i++)
    {
        vector<float> temp;
        int numRow = i * req.window;
        temp.assign(tmp.begin() + numRow, tmp.begin() + numRow + req.window);
        orinElevationSubMap.push_back(temp);
    }
    
    //构建grid map地图发布
    grid_map::GridMap map({"elevation"});
    map.setFrameId("map");
    map.setGeometry(Length(req.window * req.resolution,req.window * req.resolution),req.resolution);
    for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
    {
        const grid_map::Index index2(*iterator);
        Position position;
        map.getPosition(*iterator, position);
        map.at("elevation", *iterator) = orinElevationSubMap[index2(0)][index2(1)];
    }
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap localMap;
    GridMapRosConverter::toMessage(map,localMap);
    gridPub_.publish(localMap);
    rate.sleep();
    resp.flag = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gridmap_pub");
    GridMapToImage mapPub;
    ros::spin();
    return 0;
}

