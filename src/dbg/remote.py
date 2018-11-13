from remote.remote import Client, Command

client = Client('172.20.10.4', 5000)
client.send_command(Command.SET_MISSION, [1.8, 'kör', 'ööö'])
