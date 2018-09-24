#!/usr/bin/python2

from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker

import math
import random
import rospy

HZ = 5
STEP = 0.5

BOARD_CORNERS = [-5, 5, 5, -5]

path = Marker()

def create_obstacles():
    obst1 = Marker()
    obst2 = Marker()
    obst3 = Marker()
    obst4 = Marker()
    obst5 = Marker()

    obst1.type = obst1.CUBE
    obst2.type = obst2.CUBE
    obst3.type = obst3.CUBE
    obst4.type = obst4.CUBE
    obst5.type = obst5.CUBE

    obst1.header.frame_id = "map"
    obst2.header.frame_id = "map"
    obst3.header.frame_id = "map"
    obst4.header.frame_id = "map"
    obst5.header.frame_id = "map"

    obst1.ns = "obstacles"
    obst2.ns = "obstacles"
    obst3.ns = "obstacles"
    obst4.ns = "obstacles"
    obst5.ns = "obstacles"

    obst1.id = 0
    obst2.id = 1
    obst3.id = 2
    obst4.id = 3
    obst5.id = 4

    obst1.action = obst1.ADD
    obst2.action = obst2.ADD
    obst3.action = obst3.ADD
    obst4.action = obst4.ADD
    obst5.action = obst5.ADD

    obst1.scale.x, obst1.scale.y, obst1.scale.z = 3.0, 1.0, 1.0
    obst2.scale.x, obst2.scale.y, obst2.scale.z = 2.0, 1.0, 1.0
    obst3.scale.x, obst3.scale.y, obst3.scale.z = 3.0, 1.0, 1.0
    obst4.scale.x, obst4.scale.y, obst4.scale.z = 3.0, 1.0, 1.0
    obst5.scale.x, obst5.scale.y, obst5.scale.z = 3.0, 1.0, 1.0

    obst1.pose.position.x, obst1.pose.position.y, obst1.pose.position.z = -3.5, 1.5, 0.5
    obst2.pose.position.x, obst2.pose.position.y, obst2.pose.position.z = 0, 1.5, 0.5
    obst3.pose.position.x, obst3.pose.position.y, obst3.pose.position.z = 3.5, 1.5, 0.5
    obst4.pose.position.x, obst4.pose.position.y, obst4.pose.position.z = -2.0, -1.5, 0.5
    obst5.pose.position.x, obst5.pose.position.y, obst5.pose.position.z = 2.0, -1.5, 0.5

    obst1.pose.orientation.w = 1.0
    obst2.pose.orientation.w = 1.0
    obst3.pose.orientation.w = 1.0
    obst4.pose.orientation.w = 1.0
    obst5.pose.orientation.w = 1.0

    obst1.color.r, obst1.color.g, obst1.color.b, obst1.color.a = 0.95, 0.95, 0.95, 1.0
    obst2.color = obst1.color
    obst3.color = obst1.color
    obst4.color = obst1.color
    obst5.color = obst1.color

    obst1.lifetime = rospy.Duration()
    obst2.lifetime = rospy.Duration()
    obst3.lifetime = rospy.Duration()
    obst4.lifetime = rospy.Duration()
    obst5.lifetime = rospy.Duration()

    return [obst1, obst2, obst3, obst4, obst5]

def create_robot():
    rob = Marker()
    rob.type = rob.CUBE
    rob.header.frame_id = "map"
    rob.ns = "robot"
    rob.id = 0
    rob.action = rob.ADD
    rob.scale.x, rob.scale.y, rob.scale.z = 0.5, 0.8, 0.5
    rob.pose.position.x, rob.pose.position.y, rob.pose.position.z = -4.0, 4.0, 0.25
    rob.pose.orientation.w = 1.0
    rob.color.r, rob.color.g, rob.color.b, rob.color.a = 0.45, 0.45, 0.45, 1.0
    rob.lifetime = rospy.Duration()
    return rob

def create_target():
    tgt = Marker()
    tgt.type = tgt.CYLINDER
    tgt.header.frame_id = "map"
    tgt.ns = "target"
    tgt.id = 0
    tgt.action = tgt.ADD
    tgt.scale.x, tgt.scale.y, tgt.scale.z = 0.8, 0.8, 0.5
    tgt.pose.position.x, tgt.pose.position.y, tgt.pose.position.z = -2.5, -3.5, 0.25
    tgt.pose.orientation.w = 1.0
    tgt.color.r, tgt.color.g, tgt.color.b, tgt.color.a = 1.00, 0.83, 0.45, 1.0
    tgt.lifetime = rospy.Duration()
    return tgt

def get_vertices_structure():
    vertices = Marker()
    vertices.type = vertices.POINTS
    vertices.header.frame_id = "map"
    vertices.ns = "vertices"
    vertices.id = 0
    vertices.action = vertices.ADD
    vertices.scale.x, vertices.scale.y = 0.05, 0.05
    vertices.pose.orientation.w = 1.0
    vertices.color.r, vertices.color.g, vertices.color.b, vertices.color.a = 1.00, 0.95, 0.69, 1.00
    return vertices

def get_edges_structure():
    edges = Marker()
    edges.type = edges.LINE_LIST
    edges.header.frame_id = "map"
    edges.ns = "edges"
    edges.id = 0
    edges.action = edges.ADD
    edges.scale.x = 0.02
    edges.pose.orientation.w = 1.0
    edges.color.r, edges.color.g, edges.color.b, edges.color.a = 1.00, 0.95, 0.69, 1.00
    return edges

def get_point_structure():
    point = Marker()
    point.type = point.POINTS
    point.header.frame_id = "map"
    point.ns = "point"
    point.id = 0
    point.action = point.ADD
    point.scale.x, point.scale.y = 0.05, 0.05
    point.pose.orientation.w = 1.0
    point.color.r, point.color.g, point.color.b, point.color.a = 0.96, 0.67, 0.67, 1.00
    return point

def run_ros():
    rospy.init_node('ros_demo')

    # First param: topic name; second param: type of the message to be published; third param: size of queued messages,
    # at least 1
    chatter_pub = rospy.Publisher('some_chatter', String, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Each second, ros "spins" and draws 20 frames
    loop_rate = rospy.Rate(20) # 20hz

    frame_count = 0

    slice_index = 0

    obstacles = create_obstacles()

    robot = create_robot()

    target = create_target()

    point = get_point_structure()

    vertices = get_vertices_structure()

    edges = get_edges_structure()

    while not rospy.is_shutdown():
        msg = "Frame index: %s" % frame_count

        rospy.loginfo(msg)

        chatter_pub.publish(msg)

        for obst in obstacles:
            obst.header.stamp = rospy.Time.now()
            marker_pub.publish(obst)

        robot.header.stamp = rospy.Time.now()
        marker_pub.publish(robot)

        target.header.stamp = rospy.Time.now()
        marker_pub.publish(target)

        p0 = Point()
        p0.x, p0.y, p0.z = robot.pose.position.x, robot.pose.position.y, 0

        point.header.stamp = rospy.Time.now()

        vertices.header.stamp = rospy.Time.now()
        edges.header.stamp = rospy.Time.now()

        num_slice = 20 # e.g., to create 20 edges

        if frame_count % HZ == 0 and len(edges.points) <= 2 * num_slice:
            r_p = Point()
            r_p.x = random.uniform(BOARD_CORNERS[0], BOARD_CORNERS[1])
            r_p.y = random.uniform(BOARD_CORNERS[3], BOARD_CORNERS[2])
            r_p.z = 0
            point.points = [r_p]

            p = Point()
            angle = slice_index * 2 * math.pi / num_slice
            slice_index += 1
            p.x = STEP * math.cos(angle) + p0.x
            p.y = STEP * math.sin(angle) + p0.y
            p.z = 0
            vertices.points.append(p)  # For drawing vertices
            edges.points.append(p0)  # For drawing edges. The line list needs two points for each line
            edges.points.append(p)

        marker_pub.publish(point)
        marker_pub.publish(vertices)
        marker_pub.publish(edges)

        # TODO: Lines and Points function -> Paths function

        """"

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
        
        """

        # TODO: Robot function

        """

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
        
        """

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
    try:
        run_ros()
    except rospy.ROSInterruptException:
        pass