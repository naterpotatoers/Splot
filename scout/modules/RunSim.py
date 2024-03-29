import sys
import subprocess
import os

class RunSim:
    def __init__(self):
        # Assuming this script is run from a directory at the same level as PX4-Autopilot
        # Adjust the path to the PX4-Autopilot directory as necessary
        self.px4_autopilot_dir = os.path.join(os.path.dirname(__file__), '..', 'Firmware')

    def run(self):
        print("Running jmavsim simulation via make command")

        # Change the working directory to PX4-Autopilot to run the make command
        original_cwd = os.getcwd()  # Remember the original working directory
        try:
            # Change to the PX4-Autopilot directory
            os.chdir(self.px4_autopilot_dir)

            # Run the make command
            command = ['make', 'px4_sitl_default', 'jmavsim']
            subprocess.run(command, check=True)

        except subprocess.CalledProcessError as e:
            print(f"Failed to run simulation: {e}")
        except FileNotFoundError:
            print(f"PX4-Autopilot directory not found at expected path: {self.px4_autopilot_dir}")
        finally:
            # Change back to the original working directory
            os.chdir(original_cwd)
