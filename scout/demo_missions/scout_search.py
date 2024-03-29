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
                initial_GPS_str = line.split("Position:")[1].strip().split(",")
                WaypointGen.starting_gps = [float(coord.strip()) for coord in initial_GPS_str]

                print("GPS SET TO:", WaypointGen.starting_gps)  # Confirm GPS is set
            except ValueError as e:
                print("Error processing GPS coordinates:", e)


mission = BuildRunMission("scout_search")
print("Building mission")
mission.build()
time.sleep(3)


connection = "udp://:14540"
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
time.sleep(10)  # Adjust the delay as needed
print("System is ready")


waypoint_gen.get_gps()

waypoint_gen.add_perimeter_point([47.398170327054473, 8.5456490218639658])
waypoint_gen.add_perimeter_point([47.398058617228855, 8.5454618036746979])

# Send takeoff command
command.takeoff()
time.sleep(10)  # Wait for takeoff to complete

waypoint_gen.generate_search_mission("spiral")

time.sleep(5)
waypoint_gen.run_search_mission(command, 10.0)

time.sleep(5)

# # Add waypoints
# command.add_waypoint(47.398170327054473, 8.5456490218639658, 10.0)
# time.sleep(1)

# command.add_waypoint(47.398058617228855, 8.5454618036746979, 10.0)
# time.sleep(1)

# # Upload the mission
# command.upload_mission()
# time.sleep(1)

# # Start the mission
# command.start_mission()
# time.sleep(1)

# # Wait for a short duration before checking mission status
# time.sleep(7)

# command.pause_mission()
# time.sleep(10)

# command.add_waypoint(47.398001890458097, 8.5455576181411743, 10.0)
# time.sleep(1)

command.upload_mission()
time.sleep(1)

command.start_mission()
time.sleep(300)

# Send return to launch command
command.return_to_launch()
time.sleep(60)

# Wait for a short duration before terminating the C++ application
time.sleep(5)
cpp_process.stdin.close()
cpp_process.terminate()
cpp_process.wait()