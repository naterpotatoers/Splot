import math
import random
import time

class WaypointGen():
    def __init__(self, perimeter=[]):
        self.starting_gps = []
        self.polygon = perimeter  # A list of tuples (latitude, longitude)
        self.starting_gps_set = False
        self.current_gps = []
        self.generated_waypoints = []

    def get_gps(self):
        print(f"\nGPS coordinates: \nLatitude: {self.starting_gps[0]} \nLongitude: {self.starting_gps[1]}\n")

    def point_in_polygon(self, x, y):
        if not self.polygon:  # Return True if the polygon is empty, allowing unrestricted waypoint generation
            return True
        n = len(self.polygon)
        inside = False
        p1x, p1y = self.polygon[0]
        for i in range(n + 1):
            p2x, p2y = self.polygon[i % n]
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xints = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xints:
                            inside = not inside
            p1x, p1y = p2x, p2y
        return inside

    def generate_spiral_path(self, loops=5, initial_spacing=0.00003, spacing_increment=0.00003):
        x, y = self.starting_gps
        dx, dy = 1, 0
        current_spacing = initial_spacing
        steps_taken = 0
        steps_in_current_loop = 1
        total_steps = loops * (loops + 1)

        while steps_taken < total_steps:
            if self.point_in_polygon(x, y):
                self.generated_waypoints.append((x, y))
                steps_taken += 1
            else:
                print(f"Waypoint {x}, {y} is out of bounds.")
                break

            x += dx * current_spacing
            y += dy * current_spacing

            if steps_taken % steps_in_current_loop == 0:
                dx, dy = dy, -dx
                if steps_taken % (2 * steps_in_current_loop) == 0:
                    current_spacing += spacing_increment
                    steps_in_current_loop += 1

    def generate_grid_path(self, width=5, height=5, spacing=0.0003):
        start_x, start_y = self.starting_gps
        for x in range(width):
            for y in range(height):
                new_x = start_x + x * spacing
                new_y = start_y + y * spacing
                if self.point_in_polygon(new_x, new_y):
                    self.generated_waypoints.append((new_x, new_y))
                else:
                    print(f"Waypoint {new_x}, {new_y} is out of bounds.")

    def generate_random_path(self, num_waypoints=10, radius=0.0008):
        start_x, start_y = self.starting_gps
        for _ in range(num_waypoints):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, radius)
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)
            if self.point_in_polygon(x, y):
                self.generated_waypoints.append((x, y))
            else:
                print(f"Waypoint {x}, {y} is out of bounds.")

    def run_search_mission(self, commander, altitude_average=10.0, speed_average=5.0, RTL=True):
        for i, waypoint in enumerate(self.generated_waypoints, start=1):
            print(f"\nAdding waypoint #{i}: {waypoint}\n")
            commander.add_waypoint(waypoint[0], waypoint[1], altitude_average, speed_average)
        commander.upload_mission(RTL=RTL)
        time.sleep(1)
        commander.start_mission()
