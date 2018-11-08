"""
example:

A = Node(STOPLINE);
B = Node(STOPLINE);
R = Node(ROUNDABOUT);

course = [A, B, R]

A.addExit(R, 5)
R.addExit(A, 5)
B.addExit(R, 10)
R.addExit(B, 10)
"""

class NodeType:
    STOPLINE = 1
    PARKING = 2
    ROUNDABOUT = 3

class Command:
    IGNORE = 'ignr'
    STOP = 'stop'
    PARK = 'park'
    ENTER = 'entr'
    EXIT = 'exit'

class Node:
    def __init__(self, node_type=STOPLINE)
        self.type = node_type
        self.exits = []

    def addExit(destination, distance)
        self.exits += (destination, distance)

def closest_path(course, src, dst):
    path = [src]

    # TODO dijkstra

    return path

def create_mission(path):
    node, rest = path[0], path[1:]
    if node.type == STOPLINE:
        return [IGNORE if rest else STOP] + create_mission(rest)
    elif node.type == PARKING:
        return [IGNORE if path[1:] else PARK] + create_mission(rest)
    else: # ROUNDABOUT
        src, dst = 0, 0
        exit_count = len(node.exits)
        for i in range(exit_count):
            if node.exits[i][0] == src: src = i
            elif node.exits[i][0] == dst: dst = i
        ignore_count = (dst-src) % exit_count - 1
        return [ENTER] + stoplines*[IGNORE] + [EXIT] + create_mission(rest)
