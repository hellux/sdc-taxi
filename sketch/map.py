# skiss på kartupplägg
# räkna ut utfart i rondell med n utfarter: u = dst-src % n

class NodeType:
    STOPLINE = 1
    PARKING = 2
    ROUNDABOUT = 3

class Node:
    def __init__(self, node_type)
        self.type = node_type # NodeType
        self.exits = []

    def addExit(destination, distance)
        self.exits += (destination, distance)

A = Node(STOPLINE);
B = Node(STOPLINE);
R = Node(ROUNDABOUT);

course = [A, B, R]

A.addExit(R, 5)
R.addExit(A, 5)
B.addExit(R, 10)
R.addExit(B, 10)
