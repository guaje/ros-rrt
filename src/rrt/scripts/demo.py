#!/usr/bin/python2

from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import math
import rospy

vertices = Marker()
edges = Marker()

rob = Marker()
path = Marker()

def ros_demo(slice_index, slice_index2):
    rospy.init_node('ros_demo')

    # First param: topic name; second param: type of the message to be published; third param: size of queued messages,
    # at least 1
    chatter_pub = rospy.Publisher('some_chatter', String, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Each second, ros "spins" and draws 20 frames
    loop_rate = rospy.Rate(20) # 20hz

    frame_count = 0
    f = 0.0

    while not rospy.is_shutdown():
        # TODO: Msgs function
        msg = "Frame index: %s" % frame_count

        # Printing the message on screen
        rospy.loginfo(msg)

        # Publisher publishes messages, the msg type must be consistent with definition
        chatter_pub.publish(msg)

        # From here, we are defining and drawing two obstacles in the workspace

        # TODO: Markers function -> Maze generator function
        # Define two obstacles
        obst1 = Marker()
        obst2 = Marker()

        # Set obst1 and obst2 as a Cube and Cylinder, respectively
        obst1.type = obst1.CUBE
        obst2.type = obst2.CYLINDER

        # Set the fram ID and timestamp. See the TF tutorials for information on these
        obst1.header.frame_id = "map" # NOTE: This should be "paired" to the frame_id entry in rviz
        obst2.header.frame_id = "map" # NOTE: This should be "paired" to the frame_id entry in rviz
        obst1.header.stamp = rospy.Time.now()
        obst2.header.stamp = rospy.Time.now()

        # Set the namespace and id
        obst1.ns = "obstacles"
        obst2.ns = "obstacles"
        obst1.id = 0
        obst2.id = 1

        # Set the marker action. Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        obst1.action = obst1.ADD
        obst2.action = obst2.ADD

        # Set the scale of the marker
        obst1.scale.x, obst1.scale.y, obst1.scale.z = 1.0, 1.0, 1.0 # 1x1x1 here means each side of the cube is 1m long
        obst2.scale.x, obst2.scale.y, obst2.scale.z = 1.0, 1.0, 1.0 # 1x1x1 here means the cylinder as diameter 1m and
                                                                    # height 1m

        # Set the pose of the marker. Since a side of the obstacle obst1 is 1m as defined above, now we place the obst1
        # center at (1, 2, 0.5). z-axis is height
        obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = 1, 2, 0.5
        obst1.pose.orientation.w = 1.0 # (x, y, z, w) is a quaternion, ignore it here (quaternion can be converted back,
                                       # ros can doi it)

        obst2.pose.position.x, obst2.pose.position.y, obst2.pose.position.z = -2, -1, 0.5
        obst2.pose.orientation = obst1.pose.orientation

        # Set the color red, green, blue. If not set, by default the value is 0
        obst1.color.r, obst1.color.g, obst1.color.b, obst1.color.a = 0.0, 1.0, 0.0, 1.0 # Be sure to set alpha to
                                                                                        # something non-zero
        obst2.color = obst1.color

        obst1.lifetime = rospy.Duration()
        obst2.lifetime = rospy.Duration()

        # publish these messages to ROS system
        marker_pub.publish(obst1)
        marker_pub.publish(obst2)

        # TODO: Lines and Points function -> Paths function

        # From here, we are using points, lines, to draw a tree structure

        # We use static here since we want to incrementally add contents in these mesgs, otherwise contents in these
        # msgs will be cleaned in every ros spin
        # PYTHON: Tmp solution -> Switching to global variables

        vertices.type = vertices.POINTS
        edges.type = edges.LINE_LIST

        vertices.header.frame_id = "map"
        edges.header.frame_id = "map"
        vertices.header.stamp = rospy.Time.now()
        edges.header.stamp = rospy.Time.now()
        vertices.ns = "vertices_and_lines"
        edges.ns = "vertices_and_lines"
        vertices.action = vertices.ADD
        edges.action = edges.ADD

        vertices.pose.orientation.w = 1.0
        edges.pose.orientation.w = 1.0

        vertices.id = 0
        edges.id = 1

        # POINTS markers use x and y scale for width/height respectively
        vertices.scale.x, vertices.scale.y = 0.05, 0.05

        # LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        edges.scale.x = 0.02 # Tune it yourself

        # Points are green
        vertices.color.g, vertices.color.a = 1.0, 1.0

        # Line list is red
        edges.color.r, edges.color.a = 1.0, 1.0

        p0 = Point() # Root vertex
        p0.x, p0.y, p0.z = 0, 0, 0

        num_slice = 20 # e.g., to create 20 edges
        length = 1 # length of each edge

        herz = 10 # every 10 ROS frames we draw an edge
        if frame_count % herz == 0 and len(edges.points) <= 2 * num_slice:
            p = Point()

            angle = slice_index * 2 * math.pi / num_slice
            slice_index += 1
            p.x = length * math.cos(angle)
            p.y = length * math.sin(angle)
            p.z = 0

            vertices.points.append(p) # For drawing vertices
            edges.points.append(p0) # For drawing edges. The line list needs two points for each line
            edges.points.append(p)

        marker_pub.publish(vertices)
        marker_pub.publish(edges)

        # TODO: Robot function

        # From here, we are defining and drawing a simple robot

        rob.type = rob.SPHERE
        path.type = path.LINE_STRIP

        rob.header.frame_id = "map"
        path.header.frame_id = "map"

        rob.header.stamp = rospy.Time.now()
        path.header.stamp = rospy.Time.now()

        rob.ns = "rob"
        path.ns = "rob"

        rob.id = 0
        path.id = 1

        rob.action = rob.ADD
        path.action = path.ADD

        rob.lifetime = rospy.Duration()
        path.lifetime = rospy.Duration()

        rob.scale.x, rob.scale.y, rob.scale.z = 0.3, 0.3, 0.3

        rob.color.r, rob.color.g, rob.color.b, rob.color.a = 1.0, 0.5, 0.5, 1.0

        # Path line strip is blue
        path.color.b, path.color.a = 1.0, 1.0

        path.scale.x = 0.02
        path.pose.orientation.w = 1.0

        num_slice2 = 200 # Divide a circle into segments

        if frame_count % 2 == 0 and len(path.points) <= num_slice2:
            p = Point()

            angle = slice_index2 * 2 * math.pi / num_slice2
            slice_index2 += 1
            p.x = 4 * math.cos(angle) - 0.5 # Some random circular trajectory, with radius 4, and offset (-0.5, 1, .05)
            p.y = 4 * math.sin(angle) + 1.0
            p.z = 0.05

            rob.pose.position = p
            path.points.append(p) # For drawing path, which is line strip type

        marker_pub.publish(rob)
        marker_pub.publish(path)

        # To here, we finished displaying our components

        # Check if there is a subscriber. Here our subscriber will be Rviz
        while marker_pub.get_num_connections() < 1:
            if rospy.is_shutdown():
                return 0
            rospy.logwarn_once("Please run Rviz in another terminal.")
            rospy.sleep(1)

        loop_rate.sleep()
        frame_count += 1

if __name__ == '__main__':
    slice_index = 0
    slice_index2 = 0
    try:
        ros_demo(slice_index, slice_index2)
    except rospy.ROSInterruptException:
        pass