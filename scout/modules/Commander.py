
import requests
import time

class Commander:
    def __init__(self, cpp_process):
        self.cpp_process = cpp_process

    def takeoff(self):
        self.cpp_process.stdin.write("takeoff\n")
        self.cpp_process.stdin.flush()

    def land(self):
        self.cpp_process.stdin.write("land\n")
        self.cpp_process.stdin.flush()

    def add_waypoint(self, lat, lon, alt, spd, focus=False, dir=0.0):
        if focus:
            self.cpp_process.stdin.write(f"add_waypoint {lat} {lon} {alt} {spd} 1 {dir}\n")
        self.cpp_process.stdin.write(f"add_waypoint {lat} {lon} {alt} {spd} 0 {dir}\n")
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

    def clear_mission(self):
        self.cpp_process.stdin.write("clear\n")
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

    def return_to_splot(self, lat, lon, altitude=10.0, speed=5.0):
        self.pause_mission()
        time.sleep(1)
        self.clear_mission()
        time.sleep(1)
        # splot_coordinates = requests.get(self.splot_url + "/splot_coordinates")
        # self.cpp_process.stdin.write(f"rts {splot_coordinates.json()['latitude']} {splot_coordinates.json()['longitude']} {altitude} {speed}\n")
        self.clear_mission()
        self.add_waypoint(lat, lon, altitude, speed)
        time.sleep(1)
        self.upload_mission(RTL=False)
        time.sleep(1)
        self.start_mission()