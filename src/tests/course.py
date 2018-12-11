from remote.course import *

n1 = Node(NodeType.STOPLINE)
n2 = Node(NodeType.PARKING)

# INNER ROUNDABOUT
ri1 = Node(NodeType.INNER)
ri2 = Node(NodeType.INNER)
ri3 = Node(NodeType.INNER)
ri4 = Node(NodeType.INNER)

ro1 = Node(NodeType.OUTER)
ro2 = Node(NodeType.OUTER)
ro3 = Node(NodeType.OUTER)
ro4 = Node(NodeType.OUTER)
# ROUNDABOUT

n4 = Node(NodeType.STOPLINE)
n5 = Node(NodeType.STOPLINE)

# ROUNDABOUT INNER CIRCLE INTERNAL EDGES
ri1.addEdge(ri2, 0.5)
ri2.addEdge(ri3, 0.5)
ri3.addEdge(ri4, 0.5)
ri4.addEdge(ri1, 0.5)

# ROUNDABOUT INNER CIRCLE EXTERNAL EDGES
ri1.addEdge(n1, 20)
#ri2.addEdge(n2, 1) #WILL ADD LATER
ri3.addEdge(n5, 10)
ri4.addEdge(n4, 1)

# ROUNDABOUT OUTER CIRCLE EDGES
ro1.addEdge(ri2, 0.5)
ro2.addEdge(ri3, 0.5)
ro3.addEdge(ri4, 0.5)
ro4.addEdge(ri1, 0.5)

# OUTGOING EDGES FOR NODE 1
n1.addEdge(n2, 15)
n1.addEdge(ro1, 20)
n1.addEdge(n4, 1)

# OUTGOING EDGES FOR NODE 2
n2.addEdge(n1, 15)
n2.addEdge(ro2, 1)
n2.addEdge(n5, 1)

# OUTGOING EDGES FOR NODE 4
n4.addEdge(ro4, 1)
n4.addEdge(n1, 1)

# OUTGOING EDGES FOR NODE 5
n5.addEdge(n2, 1)
n5.addEdge(ro3, 10)


assert n1.type == NodeType.STOPLINE
assert n2.type == NodeType.PARKING
assert ri1.type == NodeType.INNER
assert ro1.type == NodeType.OUTER

assert n1.outgoing[0].cost == 15
assert n1.outgoing[0].end == n2
assert n1.outgoing[0].start == n1

assert len(n1.outgoing) == 3

course = [n5, n2, n4, n1, ri1, ri2, ri3, ri4, ro1, ro2, ro3, ro4]

path = closest_path(course, n2, n5)
mission = create_mission(path)

assert path == [n2, n5]
assert mission == [Command.STOP]

path = [n4, ro4, ri1, n1]
mission = create_mission(path)

assert mission == [Command.ENTER, Command.EXIT, Command.STOP]

path = closest_path(course, n1, n5)
mission = create_mission(path)

assert path == [n1, n4, ro4, ri1, ri2, ri3, n5]
assert mission == [Command.IGNORE, Command.ENTER, Command.CONTINUE, Command.CONTINUE, Command.EXIT, Command.STOP]

ri2.addEdge(n2, 1) # THE EDGE FROM BEFORE

path = closest_path(course, n1, n5)
mission = create_mission(path)

assert path == [n1, n4, ro4, ri1, ri2, n2, n5]
assert mission == [Command.IGNORE, Command.ENTER, Command.CONTINUE, Command.EXIT, Command.IGNORE, Command.STOP]

# CREATE EMPTY NODES
e1 = Node()
e2 = Node()

assert e1.type == NodeType.EMPTY

path = [n1, n4, ro4, ri1, ri2, n2, e1, e2, n5]
new_path = clear_empty(path)
mission = create_mission(new_path)

assert new_path == [n1, n4, ro4, ri1, ri2, n2, n5]
assert mission == [Command.IGNORE, Command.ENTER, Command.CONTINUE, Command.EXIT, Command.IGNORE, Command.STOP]