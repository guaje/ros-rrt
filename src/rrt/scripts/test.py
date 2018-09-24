from geometry_msgs.msg import Point
from tree import TreeNode, Tree

def main():
    p = Point()
    p.x, p.y, p.z = 0, 0, 0
    node = TreeNode(point=p, parent=None)
    print(node.has_any_children())

if __name__ == '__main__':
    main()