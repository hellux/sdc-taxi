from remote.course import *

n1 = Node(NodeType.STOPLINE)
n2 = Node(NodeType.PARKING)
n3 = Node(NodeType.ROUNDABOUT)

assert n1.type == NodeType.STOPLINE
assert n2.type == NodeType.PARKING
assert n3.type == NodeType.ROUNDABOUT

n4 = Node()
n5 = Node()

assert n4.type == NodeType.STOPLINE
assert n5.type == NodeType.STOPLINE

n1.addEdge(n2, 11)

assert n1.outgoing[0].cost == 11
assert n1.outgoing[0].end == n2
assert n1.outgoing[0].start == n1

n1.addEdge(n3, 20)
n1.addEdge(n4, 1)

assert len(n1.outgoing) == 3

n2.addEdge(n5, 1)

n3.addEdge(n5, 10)

n4.addEdge(n3, 1)

course = [n5, n2, n4, n1, n3]

path = closest_path(course, n1, n5)

assert len(path) == 4
assert path[0] == n1
assert path[1] == n4
assert path[2] == n3
assert path[3] == n5

n3.addEdge(n2, 1)

path = closest_path(course, n1, n5)

assert len(path) == 5
assert path[0] == n1
assert path[1] == n4
assert path[2] == n3
assert path[3] == n2
assert path[4] == n5