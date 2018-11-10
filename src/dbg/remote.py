from remote.remote import Client, Command

client = Client('127.0.0.1', 5000)
client.send_command(Command.SET_MISSION, [1.8, 'kör', 'ööö'])
