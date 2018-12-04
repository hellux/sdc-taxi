from remote.course import *

n1 = Node(NodeType.STOPLINE)
n2 = Node(NodeType.PARKING)

# ROUNDABOUT
r1 = Node(NodeType.ROUNDABOUT)
r2 = Node(NodeType.ROUNDABOUT)
r3 = Node(NodeType.ROUNDABOUT)
r4 = Node(NodeType.ROUNDABOUT)

n4 = Node()
n5 = Node()

# ROUNDABOUT INTERNAL EDGES
r1.addEdge(r2, 0.5)
r2.addEdge(r3, 0.5)
r3.addEdge(r4, 0.5)
r4.addEdge(r1, 0.5)

# OUTGOING EDGES FOR NODE 1
n1.addEdge(n2, 15)
n1.addEdge(r1, 20)
n1.addEdge(n4, 1)

# OUTGOING EDGES FOR NODE 2
n2.addEdge(n1, 15)
n2.addEdge(r2, 1)
n2.addEdge(n5, 1)

# OUTGOING EDGES FOR ROUNDABOUT
r1.addEdge(n1, 20)
# r2.addEdge(n2, 1) # WILL ADD LATER
r3.addEdge(n5, 10)
r4.addEdge(n4, 1)

# OUTGOING EDGES FOR NODE 4
n4.addEdge(r4, 1)
n4.addEdge(n1, 1)

# OUTGOING EDGES FOR NODE 5
n5.addEdge(n2, 1)
n5.addEdge(r3, 10)

assert n1.type == NodeType.STOPLINE
assert n2.type == NodeType.PARKING
assert r1.type == NodeType.ROUNDABOUT
assert n4.type == NodeType.STOPLINE

assert n1.outgoing[0].cost == 15
assert n1.outgoing[0].end == n2
assert n1.outgoing[0].start == n1

assert len(n1.outgoing) == 3

course = [n5, n2, n4, n1, r1, r2, r3, r4]

path = closest_path(course, n2, n5)
mission = create_mission(path)

assert path == [n2, n5]
assert mission == [Command.UN_PARK, Command.STOP]

path = [n4, r4, r1, n1]
mission = create_mission(path)

assert mission == [Command.ENTER, Command.EXIT, Command.STOP]

path = closest_path(course, n1, n5)
mission = create_mission(path)

assert path == [n1, n4, r4, r1, r2, r3, n5]
assert mission == [Command.IGNORE, Command.ENTER, Command.IGNORE, Command.IGNORE, Command.EXIT, Command.STOP]

r2.addEdge(n2, 1) # THE EDGE FROM BEFORE

path = closest_path(course, n1, n5)
mission = create_mission(path)

assert path == [n1, n4, r4, r1, r2, n2, n5]
assert mission == [Command.IGNORE, Command.ENTER, Command.IGNORE, Command.EXIT, Command.IGNORE, Command.STOP]

r11 = Node(NodeType.ROUNDABOUT)
r21 = Node(NodeType.ROUNDABOUT)
r31 = Node(NodeType.ROUNDABOUT)
r41 = Node(NodeType.ROUNDABOUT)

n42 = Node(NodeType.STOPLINE)

n4.clearEdges()
r1.clearEdges()
r2.clearEdges()
r3.clearEdges()
r4.clearEdges()

n4.addEdge(r11, 10)
r11.addEdge(r2, 0.5)
r2.addEdge(r21, 0)
r21.addEdge(r3, 0.5)
r3.addEdge(r31, 0)
r31.addEdge(r4, 0.5)
r4.addEdge(r41, 0)
r41.addEdge(r1, 0.5)
r1.addEdge(n42, 8)


course = [n4, n42, r1, r11, r2, r21, r3, r31, r4, r41]

path = closest_path(course, n4, n42)

assert path == [n4, r11, r2, r21, r3, r31, r4, r41, r1, n42]

mission = create_mission(path)

assert mission == [Command.ENTER, Command.IGNORE, Command.IGNORE, Command.IGNORE, Command.IGNORE, Command.IGNORE, Command.IGNORE, Command.EXIT, Command.STOP]