import math
import serial

POSITION_RADIUS_M = 100

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.is_observed = False

    def is_around(self, another):
        distanceDEG = math.sqrt(pow((another.x - self.x), 2)+pow((another.y - self.y), 2))
        distanceNM = distanceDEG * 60
        distance = distanceNM * 1852
        if (distance > POSITION_RADIUS_M):
            return False
        else:
            return True

points = [Point(34.343224, 42.342424), Point(53.3432423, 23.453545445)]

def gps_data_to_Point(data):
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

def getPosition():
    gps_serial = serial.Serial("/dev/GPS", 9600)
    satellites = 0
    while (True):
        line = str(gps_serial.readline(), encoding="ASCII")
        if "GPGGA" in line:
            data = line.split(',')
            if (int(data[7]) > 3): #enough satellites
                return(gps_data_to_Point(data))


position = Point(21.3223324, 12.3242342)

while (True):
    position = getPosition()
    print(position.x, position.y)
    for point in points:
        if position.is_around(point):
            if (point.is_observed):
                pass
            else:
                pass #delegate thread to read distance, AB:CD400 - adres AB:CD 400 pakietów - po 10 nieudanych olać