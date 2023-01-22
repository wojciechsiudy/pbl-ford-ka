import math
from serial import Serial

POSITION_RADIUS_M = 50

class Point:
    def __init__(self, x, y, address="TAG"):
        self.x = x
        self.y = y
        self.address = address
        self.is_observed = False

    def is_around(self, another):
        distance = self.get_distance_to(another)
        if (distance > POSITION_RADIUS_M):
            return False
        else:
            return True
        
    def get_distance_to(self, another):
        distanceDEG = math.acos((math.sin(another.x) * math.sin(self.x)) + (math.cos(another.x) * math.cos(self.x) * math.cos(abs(another.y - self.y))) )
        distanceNM = distanceDEG * 60
        return distanceNM * 1852

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
    while (True):
        try:
            line = str(gps_serial.readline(), encoding="ASCII")
            if "GPGGA" in line:
                data = line.split(',')
                if (int(data[7]) > 0): #enough satellites
                    return(gps_data_to_point(data))
        except(UnicodeDecodeError):
            pass

def test():
    p1 = Point(50.28838, 18.67034)
    p2 = Point(50.28817, 18.67082)
    p3 = Point(50.28050, 18.68766)
    if p1.is_around(p2):
        print("PASSED")
    else:
        print("FAILED")
    if not p1.is_around(p3):
        print("PASSED")
    else:
        print("FAILED")
