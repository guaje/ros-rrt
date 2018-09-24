class TreeNode:

    def __init__(self, point, parent):
        self.children = []
        self.point = point
        self.parent = parent

    def is_root(self):
        return not self.parent

    def has_any_children(self):
        return not self.children

class Tree:

    def __init__(self):
        self.size = 0
        self.height = 0

    def put(self):
        self.size += 1