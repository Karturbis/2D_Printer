"""program to easaly send commands to the 2D_Printer"""

#imports:
import math
from datetime import datetime
from time import sleep
import math
from os import listdir
import serial

#consts:
PORT_1 = "/dev/ttyACM0"
PORT_2 = "/dev/ttyACM1"
BAUDRATE = 115200
SERIAL_TIMEOUT = 20  # in seconds
LOGDIR = "printer_control/logs"
PRINTFILEDIR = "printer_control/print_files"
MACRODIR = "printer_control/macros"
MAX_MOVE_LENGTH = 20000

PROMPT = "input"

###################
#### Movement: ####
###################

def sign(number: int):
    return (number  > 0) - (number < 0)

def macros(arguments):
    macro_files: list = listdir(MACRODIR)
    arguments[0] = arguments[0].lower()

    if arguments[0] in macro_files:
        start_macro(arguments[0])
        with open(f"{MACRODIR}/{arguments[0]}", "r", encoding="Utf-8") as macro_reader:
            commands: list = macro_reader.readlines()
            if len(arguments) > 1:
                for i, command in enumerate(commands):
                    commands[i] = command.replace("arg0e", arguments[1])
            if len(arguments) > 2:
                for i, command in enumerate(commands):
                    commands[i] = command.replace("arg1e", arguments[2])
            if len(arguments) > 3:
                for i, command in enumerate(commands):
                    commands[i] = command.replace("arg2e", arguments[3])
            for command in commands:
                send(command)
                listen()
        end_macro()
    else:
        send("Macro not found")

def move_angle(dist:int, angle:float):
    logprint("Converting polar coordiantes to cartesian coordinates.")
    delta_x = math.cos(angle)*dist
    delta_y = math.sin(angle)*dist
    logprint(f"Real X value: {delta_x}, integer X value: {int(delta_x)}")
    logprint(f"Real Y value: {delta_y}, integer Y value: {int(delta_y)}")
    move(int(delta_x), int(delta_y))

def move(x_dist, y_dist):
    command = f"g{x_dist},{y_dist};"
    send(command)

def print_file(filename:str):
    start_macro(f"print({filename})")
    print("start opening")
    try:
        with open(f"{PRINTFILEDIR}/{filename}", "r", encoding="Utf-8") as reader:
            lines = reader.readlines()
    except FileNotFoundError as e:
        logprint(f"ERROR:Could not open the file: {e}")
        return
    print("finished opening")
    if lines[0].find(".") > 0 or lines[1].find(".") > 0:
        logprint("File uses polar coordinates")
        logprint("Coinevrting to cartesian coordinates")
        for i, line in enumerate(lines):
            lines[i] = polar_to_cartesian(line)
        logprint("Finished converting the coordinates")
    logprint("#############################")
    sleep(0.5)
    logprint("##### Starting Print ... ####")
    sleep(0.5)
    logprint("#############################")
    sleep(0.5)
    for command in lines:
        send(command)
        error = listen()
        if error:
            logprint(f"CRITICAL:Error {error} occured, stopping the print!")
            end_macro()
            return
    logprint("#############################")
    sleep(0.5)
    logprint("##### Finished Print ... ####")
    sleep(0.5)
    logprint("#############################")
    end_macro()

def polar_to_cartesian(command:str) -> str:
    if not command.startswith("g"):
        return command
    # else:
    command = command.strip("\n")
    print(command)
    angle = float(command[1:command.index(",")])
    distance = int(command[command.index(",") +1: -1])
    delta_x = int(math.cos(angle)*distance/10)
    delta_y = int(math.sin(angle)*distance/10)
    new_command = f"g{delta_x},{delta_y};"
    return new_command

###################
#### Logging: #####
###################

class Logging():

    def __init__(self):
        self.log_level: int = 1
        self.log_levels: list = ["", "DEBUG", "LOG", "WARNING", "CRITICAL"]
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

    def get_loglevels(self):
        return self.log_levels

    def get_log_level(self):
        return self.log_level

    def get_to_ignore(self):
        return self.log_levels[0:self.log_level]

    def set_log_level(self, log_level):
        try:
            log_level = int(log_level)
            self.log_level = log_level
        except TypeError:
            self.log_level = self.log_levels.index(log_level)


class Interface():

    def __init__(self):
        self.overall_command_number = 0
        self.macro_command_number = 0
        self.macro = ""
        try:
            logprint(f"Trying to connect to {PORT_1}")
            self.ser = serial.Serial(PORT_1, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT)
            logprint(f"Connected on {PORT_1}")
        except serial.serialutil.SerialException:
            logprint(f"Failed to open {PORT_1}")
            logprint(f"Trying to connect to {PORT_2}")
            try:
                self.ser = serial.Serial(PORT_2, baudrate=BAUDRATE, timeout=SERIAL_TIMEOUT)
                logprint(f"Connected on {PORT_2}")
            except serial.serialutil.SerialException:
                logprint(f"Failed to open {PORT_2}")
                logprint("Could not connect to the Printer, exiting..")
                exit(42)


    def listen(self) -> int:
        while not self.ser.in_waiting:  # wait until traffic comes in:
            pass
        reading = True
        while reading:  #read incoming traffic
            ret = self.ser.readline()
            if ret:
                decoded_ret = str(ret.decode("unicode-escape").strip("\r\n"))
                try:
                    decoded_ret = int(decoded_ret)
                    logprint(decoded_ret)
                    return decoded_ret
                except ValueError:
                    pass
                if not decoded_ret.startswith("LOOPDEBUG"):
                    logprint(decoded_ret)
            else:
                return 0

    def send(self, command:str):
        logprint(f"Sending command number {self.overall_command_number},")
        if self.macro:
            logprint(f"Macro: {self.macro}, macro command number: {self.macro_command_number},")
            self.macro_command_number += 1
        self.overall_command_number += 1
        #command = command.replace("9","N")
        logprint(f"which is: {command}")
        command = f"{command}\n".encode("ascii")
        self.ser.write(command)

    def start_macro(self, macro_name:str):
        self.macro = macro_name
        logprint("----------------------------------")
        logprint(f"## Starting macro {macro_name} ##")
        logprint("----------------------------------")

    def end_macro(self):
        self.macro = ""
        self.macro_command_number = 0
        logprint("----------------------------------")
        logprint(f"## Finished macro {self.macro} ##")
        logprint("----------------------------------")

    def disconnect(self):
        self.ser.close()

#####################
## user interface: ##
#####################


def main():
    while True:
        user_in = input("Enter command: ").split(" ")
        if user_in[0] == "exit" or user_in[0].lower() == "q":
            logprint("User abort")
            break
        elif user_in[0].startswith("#"):
            output = ""
            for i in user_in:
                output += f"{i} "
            logprint(output.strip())
            continue
        elif user_in[0].lower() == "y":
            if len(user_in) >2:
                move(user_in[2], user_in[1])
            else:
                move(0, user_in[1])
        elif user_in[0].lower() == "x":
            if len(user_in) > 2:
                move(user_in[1], user_in[2])
            else:
                move(user_in[1], 0)
        elif user_in[0].lower() == "a":
            if len(user_in) > 2:
                move_angle(user_in[1], user_in[2])
                move_angle(int(user_in[2]), float(user_in[1]))
            else:
                move_angle(user_in[1], 10000)
                move_angle(10000, float(user_in[1]))
        elif user_in[0] == "m":
            macros(user_in[1:])
            continue
        elif user_in[0].startswith("/"):
            logprint("first exit the program...")
        elif (user_in[0].startswith("g")
            or user_in[0].startswith("h")
            or user_in[0].startswith("u")
            or user_in[0].startswith("d")
            or user_in[0].startswith("c")) and user_in[0].endswith(";"):
            # give users the possibility, to send complex commands
            command = user_in[0]
            send(command)
        elif user_in[0].lower() == "p":
            print_file(user_in[1])
            continue
        elif user_in[0].lower() == "loglevel" or user_in[0].lower() == "ll":
            logging.set_log_level(user_in[1])
            continue
        else:
            logprint("Unknown command, not sending")
            continue
        listen()
    logprint("Disconecting from the Printer")
    disconnect()
    logprint("Quiting the programm")

if __name__ == "__main__":
    # init logging:
    logging = Logging()
    logprint = logging.logprint
    # init interface with Printer:
    interface = Interface()
    listen = interface.listen
    send = interface.send
    disconnect = interface.disconnect
    start_macro = interface.start_macro
    end_macro = interface.end_macro
    main()
