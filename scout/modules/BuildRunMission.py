import subprocess
import os

class BuildRunMission:
    def __init__(self, example_name):
        self.example_name = example_name
        # Assuming the script is run from within the scout/demo_missions directory
        self.project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
        self.example_dir = os.path.join(self.project_root, "MAVSDK", "examples", self.example_name)
        self.build_dir = os.path.join(self.example_dir, "build")
        self.executable_path = os.path.join(self.build_dir, self.example_name)

    def build(self):
        # Ensure the build directory exists
        os.makedirs(self.build_dir, exist_ok=True)

        # Use the -S (source) and -B (build) options which are more universally supported in CMake versions 3.13+
        subprocess.run(["cmake", "-S", self.example_dir, "-B", self.build_dir], check=True)
        
        # Build the project
        subprocess.run(["cmake", "--build", self.build_dir, "-j4"], check=True)

    def run_mission(self):
        # Define the connection URL here. Adjust as needed for your setup.
        connection_url = "udp://:14540"

        # Pass the connection URL as a command-line argument
        try:
            subprocess.run([self.executable_path, connection_url], check=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to run mission: {e}")
