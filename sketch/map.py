# skiss på kartupplägg
# räkna ut utfart i rondell med n utfarter: u = dst-src % n

class NodeType:
    STOPLINE = 1
    PARKING = 2
    ROUNDABOUT = 3

class Node:
    def __init__(self, node_type)
        self.type = node_type

class Roundabout(Node):
    def __init__(self, neighbours):
        super().__init__(ROUNDABOUT)
        self.neighbours = neighbours;

class Arc:
    def __init__(self, src, dst, distance):
        self.src = src
        self.dst = dst
        self.distance = distance

class Map:
    def __init__(self, nodes, arcs):
        self.nodes = nodes
        self.arcs = arcs
