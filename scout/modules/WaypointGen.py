import math
import random
import time

class WaypointGen():
    def __init__(self):
        self.starting_gps = []
        self.starting_gps_set = False
        self.current_gps = []
        self.generated_waypoints = []

    def get_gps(self):
        print(f"\nGPS coordinates: \nLatitude: {self.starting_gps[0]} \nLongitude: {self.starting_gps[1]}\n")

    def generate_spiral_path(self, loops=5, initial_spacing=0.00003, spacing_increment=0.00003):
        """
        Generates a spiral path from the starting GPS point.
        :param loops: Number of loops in the spiral.
        :param initial_spacing: Initial distance between each waypoint in degrees.
        :param spacing_increment: Increment added to spacing after each loop in degrees.
        """
        x, y = self.starting_gps
        dx, dy = 1, 0  # Start moving east
        current_spacing = initial_spacing
        steps_taken = 0
        steps_in_current_loop = 1
        total_steps = loops * (loops + 1)  # Total steps needed for a given number of loops

        while steps_taken < total_steps:
            # Add the current point to the waypoints
            self.generated_waypoints.append((x, y))
            steps_taken += 1

            # Move to the next point
            x += dx * current_spacing
            y += dy * current_spacing

            # Check if we need to turn
            if steps_taken % steps_in_current_loop == 0:
                # Turn right: (dx, dy) -> (dy, -dx)
                dx, dy = dy, -dx

                # After a full loop (2 turns), increase the spacing and the steps in the current loop
                if steps_taken % (2 * steps_in_current_loop) == 0:
                    current_spacing += spacing_increment
                    steps_in_current_loop += 1

    def generate_grid_path(self, width=5, height=5, spacing=0.0003):
        """
        Generates a grid path from the starting GPS point.
        :param width: Number of points in the grid's horizontal direction.
        :param height: Number of points in the grid's vertical direction.
        :param spacing: Distance between grid points in degrees.
        """
        start_x, start_y = self.starting_gps
        for x in range(width):
            for y in range(height):
                self.generated_waypoints.append((start_x + x * spacing, start_y + y * spacing))

    def generate_random_path(self, num_waypoints=10, radius=0.0008):
        """
        Generates random waypoints within a circular area around the starting point.
        :param num_waypoints: Number of waypoints to generate.
        :param radius: Radius of the circle in which to generate waypoints, in degrees.
        """
        start_x, start_y = self.starting_gps
        for _ in range(num_waypoints):
            angle = random.uniform(0, 2 * math.pi)
            distance = random.uniform(0, radius)
            x = start_x + distance * math.cos(angle)
            y = start_y + distance * math.sin(angle)
            self.generated_waypoints.append((x, y))

    def run_search_mission(self, commander, altitude_average=10.0, speed_average=5.0, RTL=True):
        for i, waypoint in enumerate(self.generated_waypoints, start=1):
            print(f"\nAdding waypoint #{i}: {waypoint}\n")
            commander.add_waypoint(waypoint[0], waypoint[1], altitude_average, speed_average)
        commander.upload_mission(RTL)
        time.sleep(1)
        commander.start_mission()
