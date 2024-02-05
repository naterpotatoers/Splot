from unittest.mock import Mock, patch
import adafruit_gps
import busio

import gps_sensor

busio.UART = Mock()


@patch('adafruit_gps.GPS')
def test_gps_sensor(mocker):
    sensor = gps_sensor.GpsSensor()
    assert sensor.gps is not None
    assert sensor.is_connected == True
    # sensor.connect()
    # assert sensor.is_connected == True
    # mocker.path('adafruit_gps.GPS.update').return_value = True
    # mocker.path('adafruit_gps.GPS.has_fix').return_value = True
    # mocker.path('adafruit_gps.GPS.latitude').return_value = 40.741895
    # mocker.path('adafruit_gps.GPS.longitude').return_value = -73.989308
    # assert sensor.read() == '{"latitude": 40.741895, "longitude": -73.989308}'