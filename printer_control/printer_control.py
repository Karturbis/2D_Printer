"""program to easaly send commands to the 2D_Printer"""

#imports:
from math import pi
from datetime import datetime
import serial

#consts:
PORT = "/dev/ttyACM1"
LOGDIR = "printer_control/logs"


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

def listen():
    while not ser.in_waiting:  # wait until traffic comes in:
        pass
    reading = True
    while reading:  #read incoming traffic
        ret = ser.readline()
        if ret:
            logprint(ret.decode("ascii").strip("\r\n"))
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


###################
#### Logging: #####
###################

class Logging():

    def __init__(self):
        self.log_name = self.generate_logname()
        with open(f"{LOGDIR}/{self.log_name}", "w", encoding="Utf-8") as f:
            f.write("Logging File\nq")

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
            # give users the possibility, to send compley commands
            command = user_in[0]
            send(command)
        else:
            logprint("Unknown command, not sending")
        listen()
    ser.close()

if __name__ == "__main__":
    logging = Logging()
    logprint = logging.logprint
    main()
