import sys

from course import Node, NodeType
from remote import Command, Client

CONN_PORT = 9849

def main():
    if len(sys.argv) < 2:
        sys.stderr.write('error: no IP address specified\n')
        return 1

    inet_addr = sys.argv[1]

    client = Client()
    try:
        client.connect(inet_addr, CONN_PORT)
    except OSError as e:
        sys.stderr.write('failed to connect to comm -- {}\n'.format(e))
        return 1

    client.send_command(Command.SET_MISSION, [1.8, 'kör', 'ööö'])

if __name__ == '__main__':
    main()
