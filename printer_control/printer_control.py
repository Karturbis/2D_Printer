"""program to easaly send commands to the 2D_Printer"""

#imports:
import serial
from math import pi

#consts:
PORT = "/dev/ttyACM0"

ser = serial.Serial(PORT, baudrate=115200, timeout=1)

def macros(arguments):
    if arguments[0].lower() == "square" or arguments[0].lower() == "s":
        dist = int(arguments[1])
        move_x(dist)
        listen()
        move_y(dist)
        listen()
        move_x(-dist)
        listen()
        move_y(-dist)

def listen():
    while not ser.in_waiting:  # wait until traffic comes in:
        pass
    reading = True
    while reading:  #read incoming traffic
        ret = ser.readline()
        if ret:
            print(ret.decode("ascii").strip("\r\n"))
        else:
            break

def send(command:str):
    print(f"Sending: {command}")
    command = f"{command}\n".encode("ascii")
    ser.write(command)

def move_x(dist:int):
    command = f"g{0.0},{dist};"
    send(command)

def move_y(dist:str):
    command = f"g{pi/2},{dist};"
    send(command)

def main():
    command_history = []

    while True:
        user_in = input("Enter command: ").split(" ")
        command_history.append(user_in)
        if user_in[0] == "exit" or user_in[0].lower() == "q":
            break
        elif user_in[0].lower() == "gy" or user_in[0].lower() == "my":
            move_y(user_in[1])
        elif user_in[0].lower() == "gx" or user_in[0].lower() == "mx":
            move_x(user_in[1])
        elif user_in[0] == "m":
            macros(user_in[1:])
        elif user_in[0].startswith("/"):
            print("first exit the program...")
        elif user_in[0].startswith(g) and user_in[0].endswith(";"):
            # give users the possibility, to send compley commands
            command = user_in[0]
            send(command)
        else:
            print("Unknown command, not sending")
        listen()
    ser.close()

if __name__ == "__main__":
    main()
