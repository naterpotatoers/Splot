import sys
import time
import threading
import subprocess
sys.path.append('../')
from modules.BuildRunMission import BuildRunMission
from modules.RunSim import RunSim

def read_from_process(process):
    while True:
        line = process.stdout.readline().strip()
        if not line:
            break
        print(f"Received from C++: {line}")

mission = BuildRunMission("scout_search")
print("Building mission")
mission.build()
time.sleep(3)

print("Running mission")

# Start the C++ application
cpp_process = subprocess.Popen(["../MAVSDK/examples/scout_search/build/scout_search"], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)

# Start a thread to continuously read the C++ application's output
threading.Thread(target=read_from_process, args=(cpp_process,), daemon=True).start()

# Wait for the system to be ready
print("Waiting for the system to be ready...")
time.sleep(10)  # Adjust the delay as needed

print("System is ready")

# Send takeoff command
cpp_process.stdin.write("takeoff\n")
cpp_process.stdin.flush()
time.sleep(10)  # Wait for takeoff to complete

# Add waypoints

cpp_process.stdin.write("add_waypoint 47.398170327054473 8.5456490218639658 10.0\n")
cpp_process.stdin.flush()
time.sleep(1)

cpp_process.stdin.write("add_waypoint 47.398241338125118 8.5455360114574432 20.0\n")
cpp_process.stdin.flush()
time.sleep(1)

# Upload the mission
cpp_process.stdin.write("upload\n")
cpp_process.stdin.flush()
time.sleep(1)

# Start the mission
cpp_process.stdin.write("start\n")
cpp_process.stdin.flush()
time.sleep(1)



# # Send land command
# print("Landing")
# cpp_process.stdin.write("land\n")
# cpp_process.stdin.flush()
# time.sleep(5)  # Wait for landing to complete

# Wait for a short duration before terminating the C++ application
time.sleep(20)

# return to launch
cpp_process.stdin.write("rtl\n")
cpp_process.stdin.flush()
time.sleep(5)  # Wait for return to launch to complete

time.sleep(10)


print("Mission Complete")

# Terminate the C++ application
cpp_process.stdin.close()
cpp_process.terminate()
cpp_process.wait()