from course import Node, NodeType
from remote import Command, Client

def main():
    client = Client('10.121.164.152', 5000)
    client.send_command(Command.SET_MISSION, [1.8, 'MADDAFAKKAAAA', 'ööö'])

if __name__ == '__main__':
    main()
