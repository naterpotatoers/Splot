import sys
import time
sys.path.append('../')
from modules.RunSim import RunSim



def run_simulation():
    print("Running simulation")
    sim_runner = RunSim()
    sim_runner.run()


run_simulation()