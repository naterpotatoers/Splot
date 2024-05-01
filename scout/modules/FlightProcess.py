import subprocess
import threading
import time
from colorama import Fore, Back, Style


class FlightProcess():
    def __init__(self, WaypointGen):
        self.WaypointGen = WaypointGen
        self.cpp_process = subprocess.Popen(["../MAVSDK/examples/scout_search/build/scout_search"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)


    def start_cpp_process(self):
        threading.Thread(target=self.read_from_process, args=(self.cpp_process,self.WaypointGen,), daemon=True).start()
        return self.cpp_process


    def read_from_process(self):
        while True:
            line = self.cpp_process.stdout.readline().strip()
            if not line:
                break
            print(Fore.YELLOW + "Received from C++:", Fore.BLUE, f"{line}", Style.RESET_ALL)
            if "Position" in line:
                initial_GPS = line.split(":")[1].split(",")
                self.WaypointGen.starting_gps = initial_GPS


    def stop_cpp_process(self):
        time.sleep(5)
        self.cpp_process.stdin.close()
        self.cpp_process.terminate()
        self.cpp_process.wait()