import math

POSITION_RADIUS_M = 100

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.is_observed = False

    def __init__(self, x, y, address):
        self.x = x
        self.y = y
        self.is_observed = False
        self.address = address

    def is_around(self, another):
        distanceDEG = math.sqrt(pow((another.x - self.x), 2)+pow((another.y - self.y), 2))
        distanceNM = distanceDEG * 60
        distance = distanceNM * 1852
        if (distance > POSITION_RADIUS_M):
            return False
        else:
            return True

points = [Point(34.343224, 42.342424), Point(53.3432423, 23.453545445)]

position = Point(21.3223324, 12.3242342)

while (True):
    #read position from gps
    for point in points:
        if position.is_around(point):
            if (point.is_observed):
                pass
            else:
                pass #delegate thread to read distance, AB:CD400 - adres AB:CD 400 pakietów