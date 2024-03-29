import time
import math
import random

class WaypointGen():
    def __init__(self):
        self.perimeter = []
        self.starting_gps = []
        self.generated_waypoint = []
        self.generated_waypoints = []

    def get_gps(self):
        print(f"\nGPS coordinates: \nLatitude: {self.starting_gps[0]} \nLongitude: {self.starting_gps[1]}\n")


    def add_perimeter_point(self, perimeter_point):
        self.perimeter.append(perimeter_point)

    def add_perimeter(self, perimeter):
        self.perimeter = perimeter

    def generate_search_mission(self, search_path):
        # TODO: Need to find how to get correct boundary coordinates from Splot server
        if search_path == "spiral":
            print("Generating a spiral search path")
            self.generate_spiral_path()
        elif search_path == "grid":
            print("Generating a grid search path")
            self.generate_grid_path()
        elif search_path == "random":
            print("Generating a random search path")
            self.generate_random_path()
        else:
            print("Invalid search path type")

    def generate_spiral_path(self):
        x, y = self.starting_gps
        dx, dy = 0, 1
        spacing = 0.001  # Adjust the spacing between waypoints as needed
        
        # while self.is_within_perimeter(x, y):
        for _ in range(20):
            self.generated_waypoints.append((x, y))
            if self.is_within_perimeter(x + dx * spacing, y + dy * spacing):
                x += dx * spacing
                y += dy * spacing
            else:
                dx, dy = -dy, dx
            _ += 1
            
    def generate_grid_path(self):
        x_min, y_min = self.perimeter[0]
        x_max, y_max = self.perimeter[1]
        spacing = 0.001  # Adjust the spacing between waypoints as needed
        
        x = x_min
        while x <= x_max:
            y = y_min
            while y <= y_max:
                if self.is_within_perimeter(x, y):
                    self.generated_waypoints.append((x, y))
                y += spacing
            x += spacing

    def generate_random_path(self):
        x_min, y_min = self.perimeter[0]
        x_max, y_max = self.perimeter[1]
        num_waypoints = 10  # Adjust the number of random waypoints as needed
        
        for _ in range(num_waypoints):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if self.is_within_perimeter(x, y):
                self.generated_waypoints.append((x, y))

    def is_within_perimeter(self, x, y):
        # x_min, y_min = self.perimeter[0]
        # x_max, y_max = self.perimeter[1]
        # return x_min <= x <= x_max and y_min <= y <= y_max
        return True

    def run_search_mission(self, commander, altitude_average):
        for i, waypoint in enumerate(self.generated_waypoints, start=1):  # 'start=0' is default and can be omitted
            print(f"\nAdding waypoint #{i}: {waypoint}\n")
            commander.add_waypoint(waypoint[0], waypoint[1], altitude_average)
            time.sleep(1)
