from gps.ka_utils import Point

def getPoints(subset):
    if subset == 1:
        return [
            Point(50.290138, 18.677277, "AA:BB"),
            Point(50.289368, 18.678203, "CC:DD")]
    else:
        return None

position = getPoints(1)[0]
msg = '%f;%f' % (position.x, position.y)
print(msg)

raw_coordinates = msg.split(';')
coordinates = Point(float(raw_coordinates[0]), float(raw_coordinates[1]))
print('Listening: %f;%f' % (coordinates.x, coordinates.y))