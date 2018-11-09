from course import Node, NodeType
from remote import Command, Client

def main():
    client = Client('10.121.162.1', 5000)
    client.send_command(Command.SET_MISSION, [1.8, 'kör', 'ööö'])

if __name__ == '__main__':
    main()
