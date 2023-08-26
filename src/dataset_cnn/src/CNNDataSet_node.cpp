#include "CNNDataSet/CNNDataSet.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"data_set");
    ros::NodeHandle nh("~");
    CNNDataSet::DataSetForCNN DataSetForCNN(nh);
    while (ros::ok())
    {
        DataSetForCNN.dataSetStart();
        ROS_INFO("One times data set is finished");
    }
    ros::spin();
    return 0;
}