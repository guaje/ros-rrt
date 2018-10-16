#!/usr/bin/python2

from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker

from tree import Tree, TreeNode

import math
import random
import rospy

HZ = 5
STEP = 1.0

BOARD_CORNERS = [-5, 5, 5, -5]


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
    tgt.type = tgt.CUBE
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


def get_tree_edges_structure():
    edges = Marker()
    edges.type = edges.LINE_LIST
    edges.header.frame_id = "map"
    edges.ns = "tree_edges"
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
    point.color.r, point.color.g, point.color.b, point.color.a = 0.45, 0.31, 0.31, 1.00
    return point


def get_collision_edges_structure():
    edges = Marker()
    edges.type = edges.LINE_LIST
    edges.header.frame_id = "map"
    edges.ns = "collision_edges"
    edges.id = 0
    edges.action = edges.ADD
    edges.scale.x = 0.02
    edges.pose.orientation.w = 1.0
    edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.96, 0.67, 0.67, 1.00
    return edges


def get_path_edges_structure():
    edges = Marker()
    edges.type = edges.LINE_LIST
    edges.header.frame_id = "map"
    edges.ns = "path_edges"
    edges.id = 0
    edges.action = edges.ADD
    edges.scale.x = 0.06
    edges.pose.orientation.w = 1.0
    edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.35, 0.75, 0.75, 1.00
    return edges


def get_obstacles_lines(obstacles):
    lines = []
    lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[2], 0)))
    lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[3], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[3], 0)))
    lines.append((Point(BOARD_CORNERS[0], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[0], BOARD_CORNERS[3], 0)))
    lines.append((Point(BOARD_CORNERS[1], BOARD_CORNERS[2], 0), Point(BOARD_CORNERS[1], BOARD_CORNERS[3], 0)))
    for obst in obstacles:
        scale_x = obst.scale.x
        scale_y = obst.scale.y
        pos_x = obst.pose.position.x
        pos_y = obst.pose.position.y
        if obst.type == obst.CUBE:  # Only rectangular obstacles are supported
            # Only aligned obstacles are supported
            x_i = pos_x - scale_x / 2
            x_f = pos_x + scale_x / 2
            y_i = pos_y - scale_y / 2
            y_f = pos_y + scale_y / 2
            lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
            lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
            lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
            lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
    return lines


def get_target_lines(target):
    lines = []
    scale_x = target.scale.x
    scale_y = target.scale.y
    pos_x = target.pose.position.x
    pos_y = target.pose.position.y
    if target.type == target.CUBE:  # Only rectangular obstacles are supported
        # Only aligned obstacles are supported
        x_i = pos_x - scale_x / 2
        x_f = pos_x + scale_x / 2
        y_i = pos_y - scale_y / 2
        y_f = pos_y + scale_y / 2
        lines.append((Point(x_i, y_i, 0), Point(x_f, y_i, 0)))
        lines.append((Point(x_i, y_f, 0), Point(x_f, y_f, 0)))
        lines.append((Point(x_i, y_i, 0), Point(x_i, y_f, 0)))
        lines.append((Point(x_f, y_i, 0), Point(x_f, y_f, 0)))
    return lines


def orientation(p, q, r):
    """
    https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    :param p:
    :param q:
    :param r:
    :return:
    """
    val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0
    return 1 if val > 0 else 2


def on_segment(p, q, r):
    """
    https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    :param p:
    :param q:
    :param r:
    :return:
    """
    if min(p.x, r.x) <= q.x <= max(p.x, r.x) and min(p.y, r.y) <= q.y <= max(p.y, r.y):
        return True
    return False


def collides_line(point_i, point_e, line):
    p_i, p_e = line
    o1 = orientation(point_i, point_e, p_i)
    o2 = orientation(point_i, point_e, p_e)
    o3 = orientation(p_i, p_e, point_i)
    o4 = orientation(p_i, p_e, point_e)
    if o1 != o2 and o3 != o4:
        return True
    if o1 == 0 and on_segment(point_i, p_i, point_e):
        return True
    if o2 == 0 and on_segment(point_i, p_e, point_e):
        return True
    if o3 == 0 and on_segment(p_i, point_i, p_e):
        return True
    if o4 == 0 and on_segment(p_i, point_e, p_e):
        return True
    return False


def collides_object(point_i, point_e, lines):
    for line in lines:
        if collides_line(point_i, point_e, line):
            return True
    return False


def run_ros():
    rospy.init_node('ros_demo')

    # First param: topic name; second param: type of the message to be published; third param: size of queued messages,
    # at least 1
    chatter_pub = rospy.Publisher('some_chatter', String, queue_size=10)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

    # Each second, ros "spins" and draws 20 frames
    loop_rate = rospy.Rate(20)  # 20hz

    frame_count = 0

    obstacles = create_obstacles()

    obstacles_lines = get_obstacles_lines(obstacles)

    robot = create_robot()

    target = create_target()

    target_lines = get_target_lines(target)

    point = get_point_structure()

    tree_edges = get_tree_edges_structure()

    p0 = Point(robot.pose.position.x, robot.pose.position.y, 0)

    tree = Tree(TreeNode(p0))

    collision_edges = get_collision_edges_structure()

    found_path = False

    path_edges = get_path_edges_structure()

    drawed_path = False

    robot_reached = False

    path_points = []

    while not rospy.is_shutdown():
        msg = "Frame index: %s" % frame_count

        rospy.loginfo(msg)

        chatter_pub.publish(msg)

        for obst in obstacles:
            obst.header.stamp = rospy.Time.now()
            marker_pub.publish(obst)

        robot.header.stamp = rospy.Time.now()

        target.header.stamp = rospy.Time.now()
        marker_pub.publish(target)

        point.header.stamp = rospy.Time.now()

        tree_edges.header.stamp = rospy.Time.now()

        collision_edges.header.stamp = rospy.Time.now()

        path_edges.header.stamp = rospy.Time.now()

        if frame_count % HZ == 0 and not found_path:
            rand_pnt = Point()
            rand_pnt.x = random.uniform(BOARD_CORNERS[0], BOARD_CORNERS[1])
            rand_pnt.y = random.uniform(BOARD_CORNERS[3], BOARD_CORNERS[2])
            rand_pnt.z = 0
            point.points = [rand_pnt]

            close_node = tree.get_closest_node(rand_pnt)
            close_pnt = close_node.point

            # https://math.stackexchange.com/questions/175896/finding-a-point-along-a-line-a-certain-distance-away-from-another-point

            total_dist = math.sqrt(math.pow(rand_pnt.x - close_pnt.x, 2) + math.pow(rand_pnt.y - close_pnt.y, 2))

            dist_ratio = STEP / total_dist

            new_pnt = Point()
            new_pnt.x = (1 - dist_ratio) * close_pnt.x + dist_ratio * rand_pnt.x
            new_pnt.y = (1 - dist_ratio) * close_pnt.y + dist_ratio * rand_pnt.y
            new_pnt.z = 0

            if collides_object(close_pnt, new_pnt, obstacles_lines):
                collision_edges.points.append(close_pnt)
                collision_edges.points.append(new_pnt)
            else:
                last_node = tree.add_node(close_node, new_pnt)

                tree_edges.points.append(close_pnt)
                tree_edges.points.append(new_pnt)

            if collides_object(close_pnt, new_pnt, target_lines):
                found_path = True

        if found_path and not drawed_path:
            current_node = last_node
            while not current_node.is_root():
                path_points.append(current_node.point)
                path_edges.points.append(current_node.point)
                path_edges.points.append(current_node.parent.point)
                current_node = current_node.parent
            drawed_path = True

        if frame_count % 2 == 0 and drawed_path and not robot_reached:
            robot.pose.position = path_points.pop()
            robot_reached = True if len(path_points) == 0 else False

        marker_pub.publish(robot)
        marker_pub.publish(point)
        marker_pub.publish(tree_edges)
        marker_pub.publish(collision_edges)
        marker_pub.publish(path_edges)

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
