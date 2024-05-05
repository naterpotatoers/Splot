import sys
import time
import threading
import requests
import json
import subprocess
sys.path.append('../')
from modules.Commander import Commander
from modules.BuildRunMission import BuildRunMission
from modules.RunSim import RunSim
from colorama import Fore, Back, Style
from modules.WaypointGen import WaypointGen

drone_ID = "scout_1"
backend_url = "http://localhost:5001"
desc = "Scout 1 at this location"

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
                    
                    # Send GPS coordinates to the backend
                    coords = {"lat": WaypointGen.current_gps[0], "lng": WaypointGen.current_gps[1]}
                    scout_data = {"id": f"{drone_ID}", "coords": coords, "desc": f"{desc}"}
                    requests.post(f"{backend_url}/explored", json=scout_data)

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

# parse the perimeter data that is a list into a list of tuples
try:
    response = requests.get(f"{backend_url}/perimeter").json()
    perimeter = [(point['coords']['lat'], point['coords']['lng']) for point in response]
    print("Perimeter data:", perimeter)
except requests.exceptions.RequestException as e:
    print(f"Request failed: {e}")
except KeyError:
    print("Error in data format: expected keys not found")
except ValueError:
    print("JSON parsing error")


# Create a Commander object to send commands to the C++ application
command = Commander(cpp_process)
waypoint_gen = WaypointGen(perimeter=[])
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


command.add_waypoint(47.3978897, 8.54551, 10.0, 5.0, True, 270.0)
command.add_waypoint(47.397887999999995, 8.545460199999999, 10.0, 5.0, True, 90.0)
command.add_waypoint(47.3977709, 8.5455683, 10.0, 5.0, False)
command.add_waypoint(47.3978876, 8.5454683, 10.0, 5.0, False)


# waypoint_gen.generate_spiral_path()
# waypoint_gen.generate_grid_path()
# waypoint_gen.generate_random_path()

time.sleep(5)
# waypoint_gen.run_search_mission(command, 10.0, 2.0, RTL=True)

command.upload_mission(RTL=True)
time.sleep(1)
command.start_mission()

# while True:
#     command.get_status()
#     time.sleep(25)
#     break
# command.return_to_splot(47.398170327054473, 8.5456490218639658)

while True:
    command.get_status()
    time.sleep(1)