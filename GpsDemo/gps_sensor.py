import time
# import board
# import busio

# https://docs.circuitpython.org/projects/gps/en/latest/
import adafruit_gps

class GpsSensor:
    is_connected = False

    timeout = 30
    # RX = board.RX
    # TX = board.TX
    gps = None
    uart = None
    last_print = 0
    baudRate = 115200

    def __init__(self):
        print("Initializing GPS device")
        # self.uart = busio.UART(self.TX, self.RX, baudrate=self.baudRate, timeout=self.timeout)
        # self.gps = adafruit_gps.GPS(self.uart, debug=False)

    def connect(self):
        print("Connecting to GPS device")
        # self.gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        # self.gps.send_command(b'PMTK220,1000')
        self.is_connected = True

    def read(self):
        # self.gps.update()
        current_time = 1 # replace w/ time.monotonic()
        time_difference = current_time - self.last_print
        if (time_difference >= 1.0) and self.is_connected:
            self.last_print = current_time
            # if not self.gps.has_fix:
            #     print('Waiting for GPS fix...')
            #     return ""
            mock_latitude = 40.741895
            mock_longitude = -73.989308
            return '{{"latitude": {0:.6f}, "longitude": {1:.6f}}}'.format(mock_latitude, mock_longitude)

    def close(self):
        print("Closing GPS device")
        self.is_connected = False

