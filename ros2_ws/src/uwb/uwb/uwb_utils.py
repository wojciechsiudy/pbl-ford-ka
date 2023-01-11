
from serial import Serial

baudrate = 115200

def getDistanceArray(serialPath, address, size):
    uwb_serial = Serial(serialPath, baudrate)
    distances = []
    message = address + str(size)
    uwb_serial.write(bytes(message, "ASCII"))
    while (size > 0):
        size -= 1
        line = str(uwb_serial.readline(), encoding="ASCII")
        try:
            distance = float(line[8:].strip())
            distances.append(distance)
        except(ValueError): #in case of reciver failure
            pass
    return distances

def getAverage(distances):
    if len(distances) == 1:
        return
    sum = 0
    n = 0
    for distance in distances:
        sum += distance
        n += 1
    if (n == 0):
        return 0 #error
    else:
        return (sum / n)

print(getAverage(getDistanceArray("/dev/UWBt", "AA:BB", 3)))