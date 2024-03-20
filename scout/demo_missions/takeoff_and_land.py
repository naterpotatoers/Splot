import sys
import time
import threading
sys.path.append('../')

from modules.BuildRunMission import BuildRunMission
from modules.RunSim import RunSim

# def run_simulation():
#     print("Running simulation")
#     sim_runner = RunSim()
#     sim_runner.run()
#     print("Simulation ended")

mission = BuildRunMission("offboard")

print("Building mission")
mission.build()
time.sleep(3)

# Start the simulation in a separate thread
# simulation_thread = threading.Thread(target=run_simulation)
# simulation_thread.start()

# Wait a bit for the simulator to initialize properly
# time.sleep(20)

print("Running mission")
# Call the function to run the mission
mission.run_mission()
