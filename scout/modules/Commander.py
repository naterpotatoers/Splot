class Commander:
    def __init__(self, cpp_process):
        self.cpp_process = cpp_process

    def takeoff(self):
        self.cpp_process.stdin.write("takeoff\n")
        self.cpp_process.stdin.flush()


    def land(self):
        self.cpp_process.stdin.write("land\n")
        self.cpp_process.stdin.flush()

    def add_waypoint(self, lat, lon, alt, spd):
        self.cpp_process.stdin.write(f"add_waypoint {lat} {lon} {alt} {spd}\n")
        self.cpp_process.stdin.flush()

    def upload_mission(self, RTL=True):
        if RTL:
            self.cpp_process.stdin.write("upload\n")
        else:
            self.cpp_process.stdin.write("upload_no_rtl\n")
        self.cpp_process.stdin.flush()

    def start_mission(self):
        self.cpp_process.stdin.write("start\n")
        self.cpp_process.stdin.flush()

    def pause_mission(self):
        self.cpp_process.stdin.write("pause\n")
        self.cpp_process.stdin.flush()

    def resume_mission(self):
        self.cpp_process.stdin.write("resume\n")
        self.cpp_process.stdin.flush()

    def return_to_launch(self):
        self.cpp_process.stdin.write("rtl\n")
        self.cpp_process.stdin.flush()

    def get_status(self):
        self.cpp_process.stdin.write("status\n")
        self.cpp_process.stdin.flush()


