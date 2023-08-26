#!/usr/bin/env python
import rospy
import pandas as pd
from visualization_msgs.msg import Marker, MarkerArray
import rospkg
from geometry_msgs.msg import Point


def main():
    rospy.init_node('path')
    rospack = rospkg.RosPack()
    rospy.loginfo('Publishing Path')
    file_path = rospack.get_path("prm_apf_planner")
    publisher = rospy.Publisher('/paths', MarkerArray, queue_size=1)
    data = pd.read_csv(file_path + '/results/data.csv')
    Y = data['x'].values
    X = data['y'].values
    Xn = len(X) + 17 - Y
    Yn = X

    X = Xn
    Y = Yn
    markerArray = MarkerArray()
    z = 1.5
    while not rospy.is_shutdown():
        marker_start = Marker()
        marker_start.header.frame_id = "/map"
        marker_start.type = marker_start.SPHERE
        marker_start.action = marker_start.ADD
        marker_start.scale.x = 0.5
        marker_start.scale.y = 0.5
        marker_start.scale.z = 0.5
        marker_start.color.a = 1.0
        marker_start.color.g = 1.0
        marker_start.pose.orientation.w = 1.0
        marker_start.pose.position.x = (X[0] / 5.0) - 10.0
        marker_start.pose.position.y = 10.0 - (Y[0] / 5.0)
        marker_start.pose.position.z = z
        markerArray.markers.append(marker_start)

        marker_goal = Marker()
        marker_goal.header.frame_id = "/map"
        marker_goal.type = marker_goal.SPHERE
        marker_goal.action = marker_goal.ADD
        marker_goal.scale.x = 0.5
        marker_goal.scale.y = 0.5
        marker_goal.scale.z = 0.5
        marker_goal.color.a = 1.0
        marker_goal.color.r = 1.0
        marker_goal.pose.orientation.w = 1.0
        marker_goal.pose.position.x = (X[-1] / 5.0) - 10.0
        marker_goal.pose.position.y = 10.0 - (Y[-1] / 5.0)
        marker_goal.pose.position.z = z
        markerArray.markers.append(marker_goal)

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.03
        marker.color.a = 1.0
        # marker.color.b = 1.0
        marker.pose.orientation.w = 1.0
        marker.points = []
        for i in range(len(X) - 1):
            f_ = str(i)
            f_ = Point()
            f_.x = (X[i] / 5.0) - 10.0
            f_.y = 10.0 - (Y[i] / 5.0)
            f_.z = z
            marker.points.append(f_)
            s_ = str(i + 1)
            s_ = Point()
            s_.x = (X[i + 1] / 5.0) - 10.0
            s_.y = 10.0 - (Y[i + 1] / 5.0)
            s_.z = z
            marker.points.append(s_)
        markerArray.markers.append(marker)

        id = 0
        for m in markerArray.markers:
            m.id = id
            id += 1
        publisher.publish(markerArray)
        rospy.sleep(0.5)


if __name__ == '__main__':
    main()
