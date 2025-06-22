"""program to easaly send commands to the 2D_Printer"""

#imports:
from math import pi
from datetime import datetime
from time import sleep
import serial

#consts:
PORT = "/dev/ttyACM0"
LOGDIR = "printer_control/logs"
PRINTFILEDIR = "printer_control/print_files"
MAX_MOVE_LENGTH = 20000

###################
#### Movement: ####
###################

def sign(number: int):
    return (number  > 0) - (number < 0)

def macros(arguments):
    if arguments[0].lower() == "square" or arguments[0].lower() == "s":
        dist = int(arguments[1])
        move(dist, 0.0)
        listen()
        move(dist, pi/2)
        listen()
        move(-dist, 0.0)
        listen()
        move(-dist, pi/2)
    elif arguments[0].lower() == "t0":
        send("g5.7,20000;")
    elif arguments[0].lower() == "-t0":
        send("g5.7,-20000;")

def move(dist:int, angle:float):
    dist = int(dist)
    angle = float(angle)
    sign_dist = sign(dist)
    dist = abs(dist)
    if dist < MAX_MOVE_LENGTH:
        command = f"g{angle},{sign_dist * dist};"
    else:
        floor_div = dist//MAX_MOVE_LENGTH
        command = f"g{angle},{sign_dist * MAX_MOVE_LENGTH};"
        for _ in range(floor_div):
            send(command)
            listen()
        command = f"g{angle},{sign_dist * (dist%MAX_MOVE_LENGTH)};"
    send(command)

def print_file(filename:str):
    send("h;")
    listen()
    try:
        with open(f"{PRINTFILEDIR}/{filename}", "r", encoding="Utf-8") as reader:
            lines = reader.readlines()
    except FileNotFoundError as e:
        logprint(f"ERROR:Could not open the file: {e}")
        return
    logprint("#############################")
    sleep(0.5)
    logprint("##### Starting Print ... ####")
    sleep(0.5)
    logprint("#############################")
    sleep(0.5)
    for command in lines:
        if not command.startswith("h"):
            send(command)
            listen()
    logprint("#############################")
    sleep(0.5)
    logprint("##### Finished Print ... ####")
    sleep(0.5)
    logprint("#############################")

###################
#### Logging: #####
###################

class Logging():

    def __init__(self):
        self.log_name = self.generate_logname()
        with open(f"{LOGDIR}/{self.log_name}", "w", encoding="Utf-8") as f:
            f.write(f"--------------------------------------\nLOG FILE {self.log_name}\n--------------------------------------\n")

    def generate_logname(self):
        time_raw = str(datetime.now())
        time_formatted = time_raw.replace("-", "_").replace(" ", "_").replace(":", "-")[:18]
        return f"{time_formatted}.LOG"

    def logprint(self, data:str):
        with open(f"{LOGDIR}/{self.log_name}", "a", encoding="Utf-8") as writer:
            time_now = str(datetime.now())
            writer.writelines(f"LOG({time_now[11:]}): {data}\n")
        print(data)

class Interface():

    def __init__(self):
        self.command_number = 0
        self.ser = serial.Serial(PORT, baudrate=115200, timeout=1)

    def listen(self):
        while not self.ser.in_waiting:  # wait until traffic comes in:
            pass
        reading = True
        while reading:  #read incoming traffic
            ret = self.ser.readline()
            if ret:
                decoded_ret = ret.decode("ascii").strip("\r\n")
                if not decoded_ret.startswith("LOOP"):
                    logprint(decoded_ret)
                if decoded_ret == "0":
                    break
            else:
                break

    def send(self, command:str):
        logprint(f"Sending command number {self.command_number},")
        self.command_number += 1
        logprint(f"which is: {command}")
        command = f"{command}\n".encode("ascii")
        self.ser.write(command)
    
    def disconnect(self):
        self.ser.close()


def main():
    command_history = []
    while True:
        user_in = input("Enter command: ").split(" ")
        command_history.append(user_in)
        if user_in[0] == "exit" or user_in[0].lower() == "q":
            break
        elif user_in[0].startswith("#"):
            output = ""
            for i in user_in:
                output += f"{i} "
            logprint(output.strip())
            continue
        elif user_in[0].lower() == "y":
            move(user_in[1], pi/2)
        elif user_in[0].lower() == "x":
            move(user_in[1], 0.0)
        elif user_in[0].lower() == "a":
            if len(user_in) > 2:
                move(user_in[2], user_in[1])
            else:
                move(10000, user_in[1])
        elif user_in[0] == "m":
            macros(user_in[1:])
        elif user_in[0].startswith("/"):
            logprint("first exit the program...")
        elif (user_in[0].startswith("g")
            or user_in[0].startswith("h")
            or user_in[0].startswith("u")
            or user_in[0].startswith("d")) and user_in[0].endswith(";"):
            # give users the possibility, to send complex commands
            command = user_in[0]
            send(command)
        elif user_in[0].lower() == "p":
            print_file(user_in[1])
            continue
        else:
            logprint("Unknown command, not sending")
            continue
        listen()
    disconnect()

if __name__ == "__main__":
    # init logging:
    logging = Logging()
    logprint = logging.logprint
    # inti interface with Printer:
    interface = Interface()
    listen = interface.listen
    send = interface.send
    disconnect = interface.disconnect
    main()
