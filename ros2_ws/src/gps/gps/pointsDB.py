from gps.ka_utils import Point

def getPoints(subset):
    if subset == 1:
        return [
            Point(50.277863, 18.669865, "AA:BB"),
            Point(50.277863, 11.669865, "DALEKI")]
    else:
        return None

position = getPoints(1)[0]
msg = '%f;%f' % (position.x, position.y)
print(msg)

raw_coordinates = msg.split(';')
coordinates = Point(float(raw_coordinates[0]), float(raw_coordinates[1]))
print('Listening: %f;%f' % (coordinates.x, coordinates.y))