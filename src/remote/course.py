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
import heapq

class NodeType:
    STOPLINE = 1
    PARKING = 2
    ROUNDABOUT = 3
    NULL = 4

class Command:
    IGNORE = 'ignr'
    STOP = 'stop'
    PARK = 'park'
    ENTER = 'entr'
    EXIT = 'exit'

class Node:
    def __init__(self, node_type=NodeType.STOPLINE):
        self.type = node_type
        self.exits = []

    def addExit(self, destination, distance):
        self.exits += (destination, distance)

class Edge:
    def __init__(self, dist=0 , dest=Node(NodeType.NULL)):
        self.dist = dist
        self.dest = dest

    def __lt__(self, other):
        return self.dist < other.dist

def closest_path(course, src, dst):
    inf = float('inf')
    path = [src]
    q = []
    unvisited = []

    for node in course:
        if node != src:
            heapq.heappush(unvisited, (inf, node, None))
        else:
            heapq.heappush(unvisited, (0, src, None))

    heapq.heapify(unvisited)
    while(unvisited):
        cur = unvisited.pop()
        for exit in cur[1].exits:
            if 
            

    # TODO dijkstra

    return path

def create_mission(path):
    node, rest = path[0], path[1:]
    if node.type is NodeType.STOPLINE:
        return ([Command.IGNORE] if rest else [Command.STOP]) + create_mission(rest)
    elif node.type is NodeType.PARKING:
        return ([Command.IGNORE] if rest else [Command.PARK]) + create_mission(rest)
    else: # ROUNDABOUT
        src, dst = 0, 0
        exit_count = len(node.exits)
        for i in range(exit_count):
            if node.exits[i][0] == src: src = i
            elif node.exits[i][0] == dst: dst = i
        ignore_count = (dst-src) % exit_count - 1
        return [Command.ENTER] + ignore_count*[Command.IGNORE] + [Command.EXIT] + create_mission(rest)
