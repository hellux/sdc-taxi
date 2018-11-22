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
        self.edges = []

    def addExit(self, dest, cost):
        self.edges.append(Edge(self, dest, cost))

class Edge:
    def __init__(self, start = None, end = None, cost=0):
        self.start = start
        self.end = end
        self.cost = cost
        
        start.addExit(self, end, cost)

    def __lt__(self, other):
        return self.cost < other.cost
        
class PathPart:
    def __init__(self, cur=None, prev=None, cost=float('inf')):
        self.cur = cur
        self.prev = prev
        self.cost = cost

    def __lt__(self, other):
        return self.cost < other.cost
        

def closest_path(course, src, dst):
    inf = float('inf')
    path = [src]

    q = []

    unvisited = []
    visited = []

    for node in course:
        if node != src:
            heapq.heappush(unvisited, PathPart(node))
        else:
            heapq.heappush(unvisited, PathPart(src, cost=0)

    while(unvisited):
        cur = unvisited.pop()
        for part in unvisited():
            for exit in cur.exits:
                if exit.dest == part.cur:
                    if cur.cost + exit.dist < part.cost:
                        part.cost = cur.cost + exit.dist

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
