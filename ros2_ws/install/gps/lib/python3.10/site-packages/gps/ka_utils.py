import math
from serial import Serial

POSITION_RADIUS_M = 100

class Point:
    def __init__(self, x, y, address="TAG"):
        self.x = x
        self.y = y
        self.address = address
        self.is_observed = False

    def is_around(self, another):
        distanceDEG = math.sqrt(pow((another.x - self.x), 2)+pow((another.y - self.y), 2))
        distanceNM = distanceDEG * 60
        distance = distanceNM * 1852
        if (distance > POSITION_RADIUS_M):
            return False
        else:
            return True

def gps_data_to_point(data):
    lat_raw = data[2]
    lat_deg = float(lat_raw[0:2])
    lat_min = float(lat_raw[2:])
    lat = lat_deg + (lat_min / 60)
    sign_lat = data[3]
    if sign_lat == "S":
        lat *= -1

    long_raw = data[4]
    long_deg = float(long_raw[0:3])
    long_min = float(long_raw[3:])
    long = long_deg + (long_min / 60)
    long_sign = data[5]
    if long_sign == "W":
        long *= -1

    return Point(lat, long)

def get_position():
    gps_serial = Serial("/dev/GPS")
    satellites = 0
    while (True):
        try:
            line = str(gps_serial.readline(), encoding="ASCII")
            if "GPGGA" in line:
                data = line.split(',')
                if (int(data[7]) > 0): #enough satellites
                    return(gps_data_to_point(data))
        except(UnicodeDecodeError):
            pass
       