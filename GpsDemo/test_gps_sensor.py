import gps_sensor

def test_gps_sensor():
    sensor = gps_sensor.GpsSensor()
    sensor.connect()
    data = sensor.read()
    assert data == '{"latitude": 40.741895, "longitude": -73.989308}'