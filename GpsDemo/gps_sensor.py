import time
import busio

# https://docs.circuitpython.org/projects/gps/en/latest/
import adafruit_gps

import mock_adafruit_gps

class GpsSensor:
    gps = None
    last_print = time.monotonic()
    is_connected = False #is this public or private? 
    # a: private how do i make it public?


    def __init__(self):
        print("Initializing GPS device")
        TX = 14
        RX = 15
        uart = busio.UART(TX, RX, baudrate=115200)
        self.gps = mock_adafruit_gps.GPS(uart, debug=False)

    def connect(self):
        print("Connecting to GPS device")
        self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0') # enables RMC and GGA
        self.gps.send_command(b'PMTK220,1000') # update rate of 1Hz
        self.is_connected = True

    def read(self) -> str:
        self.gps.update()
        current_time = time.monotonic()
        time_difference = current_time - self.last_print
        if (time_difference >= 1.0) and self.is_connected:
            self.last_print = current_time
            if not self.gps.has_fix:
                print('Waiting for GPS fix...')
                return ""
            lat = self.gps.latitude
            lon = self.gps.longitude
            return f'{{"latitude": 40.741895, "longitude": -73.989308}}'
        return "gegad"

    def close(self):
        print("Closing GPS device")
        self.gps.send_command(b'PMTK314,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.is_connected = False

    def is_connected(self) -> bool:
        return self.is_connected
