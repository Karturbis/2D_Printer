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


ser = serial.Serial(PORT, baudrate=115200, timeout=1)

###################
#### Movement: ####
###################

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
    elif arguments[0].lower() == "t0":
        send("g5.7,20000;")
    elif arguments[0].lower() == "-t0":
        send("g5.7,-20000;")

def listen():
    while not ser.in_waiting:  # wait until traffic comes in:
        pass
    reading = True
    while reading:  #read incoming traffic
        ret = ser.readline()
        if ret:
            decoded_ret = ret.decode("ascii").strip("\r\n")
            if not decoded_ret.startswith("LOOP"):
                logprint(decoded_ret)
            if decoded_ret == 0:
                break
        else:
            break

def send(command:str):
    logprint(f"Sending: {command}")
    command = f"{command}\n".encode("ascii")
    ser.write(command)

def move_x(dist:int):
    command = f"g{0.0},{dist};"
    send(command)

def move_y(dist:str):
    command = f"g{pi/2},{dist};"
    send(command)

def print_file(filename:str):
    send("h;")
    listen()
    print("homed")
    for _ in range(10):
        move_x(-20000)
        listen()
    print("moved")
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
        elif user_in[0].lower() == "gy" or user_in[0].lower() == "my":
            move_y(user_in[1])
        elif user_in[0].lower() == "gx" or user_in[0].lower() == "mx":
            move_x(user_in[1])
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
    ser.close()

if __name__ == "__main__":
    logging = Logging()
    logprint = logging.logprint
    main()
