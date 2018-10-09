# mission commands activated by stoplines

def ignore(t):
    pass

def enter(t):
    t.turn_right()

def exit(t):
    t.turn_right()

def stop(t):
    t.stop()

def park(t):
    t.park()

commands = []

def signal_stopline()
    commands.pop(0)(taxi)
