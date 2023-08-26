#include "ros/ros.h"
#include <ros/time.h>

class Timer_test
{
private:
    ros::NodeHandle nh;
    ros::Timer timer;
public:
    Timer_test(/* args */);
    void timer_test();
    void doSomeThing(const ros::TimerEvent &event);
    ~Timer_test();
};

Timer_test::Timer_test(/* args */)
{
}

void Timer_test::doSomeThing(const ros::TimerEvent &event)
{
    ROS_INFO("Timer is succeed!");
}

void Timer_test::timer_test()
{
    timer = nh.createTimer(ros::Duration(0.5), &Timer_test::doSomeThing, this);
}


Timer_test::~Timer_test()
{
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"timer_test");
    Timer_test test;
    test.timer_test();
    ros::spin();
    return 0;
}
