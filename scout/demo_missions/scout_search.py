import sys
import time
import threading
import subprocess
sys.path.append('../')
from modules.Commander import Commander
from modules.BuildRunMission import BuildRunMission
from modules.RunSim import RunSim
from colorama import Fore, Back, Style
from modules.WaypointGen import WaypointGen

def print_cpp(line):
    print(Fore.YELLOW + "Received from C++:", Fore.BLUE, f"{line}", Style.RESET_ALL)

def read_from_process(process, WaypointGen):
    while True:
        # Directly read the line without decoding
        line = process.stdout.readline().strip()
        if not line:
            break
        print_cpp(line)

        if "Position" in line:
            try:
                # Extracting GPS coordinates, stripping spaces, and converting to floats
                gps_str = line.split("Position:")[1].strip().split(",")
                gps_coordinates = [float(coord.strip()) for coord in gps_str]

                # Check if starting GPS is set, if not, set it
                if not WaypointGen.starting_gps_set:
                    WaypointGen.starting_gps = gps_coordinates
                    WaypointGen.starting_gps_set = True  # Mark the starting GPS as set
                    print("Starting GPS SET TO:", WaypointGen.starting_gps)
                else:
                    # Update current GPS or do other processing
                    WaypointGen.current_gps = gps_coordinates
                    print("\nCurrent GPS updated to:", WaypointGen.current_gps)
            except ValueError as e:
                print("Error processing GPS coordinates:", e)

        if "Landing" in line:
            print(Fore.RED + "Landing detected")
            time.sleep(10)
            process.stdin.close()
            process.terminate()
            process.wait()

mission = BuildRunMission("scout_search")
print("Building mission")
mission.build()
time.sleep(3)

connection = "udp://:14540" # For SITL Connection
# connection = "serial:///dev/tty.usbserial-01E98097:57600" # For Radio Connection
print("Running mission")
# Start the C++ application
cpp_process = subprocess.Popen(["../MAVSDK/examples/scout_search/build/scout_search", connection], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

# Create a Commander object to send commands to the C++ application
command = Commander(cpp_process)
waypoint_gen = WaypointGen()
# Start a thread to continuously read the C++ application's output
threading.Thread(target=read_from_process, args=(cpp_process,waypoint_gen,), daemon=True).start()

# Wait for the system to be ready
print("Waiting for the system to be ready...")
time.sleep(5)  # Adjust the delay as needed
print("System is ready")


waypoint_gen.get_gps()

# Send takeoff command
command.takeoff()
time.sleep(10)  # Wait for takeoff to complete

waypoint_gen.generate_spiral_path()
# waypoint_gen.generate_grid_path()
# waypoint_gen.generate_random_path()

time.sleep(5)
waypoint_gen.run_search_mission(command, 10.0, 2.0, RTL=True)

while True:
    command.get_status()
    time.sleep(25)
    break
command.return_to_splot(47.398170327054473, 8.5456490218639658)

while True:
    command.get_status()
    time.sleep(1)