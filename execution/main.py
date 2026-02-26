import socket
import time
import csv


def send_tcp_packet(client_socket, message):
    """
    Send a message to the server and receive the response.
    """
    try:
        client_socket.sendall(message.encode('utf-8'))
        response = client_socket.recv(1024).decode('utf-8')
        print(f"Server response: {response}")
        return response
    except socket.error as e:
        print(f"Socket error: {e}")
        return None


def read_angles_from_csv(file_path):
    angles = []
    try:
        with open(file_path, mode='r') as file:
            csv_reader = csv.reader(file)
            for row in csv_reader:
                # Convert each angle to a float and append to angles list
                angles.append([float(angle) for angle in row])
        return angles
    except FileNotFoundError:
        print(f"Error: File '{file_path}' not found.")
        return []
    except ValueError as e:
        print(f"Error: Invalid data in CSV file - {e}")
        return []



# Server details
SERVER_IP = '192.168.1.159'
SERVER_PORT = 5001

# File containing joint angles
csv_file_path = 'angles_maze.csv'

# Read angles from the CSV file
angles = read_angles_from_csv(csv_file_path)

if not angles:
    print("No angles to send. Exiting.")

while True:# Establish the TCP connection once
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((SERVER_IP, SERVER_PORT))
            print(f"Connected to server at {SERVER_IP}:{SERVER_PORT}")

            for angle_set in angles:
                # Round angles to 2 decimal places
                rounded_angles = [round(angle, 2) for angle in angle_set]
                # Prepare the message
                message = f"set_angles({', '.join(map(str, rounded_angles))}, 20)"
                # Send the message and receive the response
                send_tcp_packet(client_socket, message)
                time.sleep(1)

    except socket.error as e:
        print(f"Error: Could not connect to server - {e}")



