#def get_position():
#    gps_serial = Serial("/dev/GPS")
#    while (True):
#        try:
#            line = str(gps_serial.readline(), encoding="ASCII")
#            if "GPGGA" in line:
#                data = line.split(',')
#                if (int(data[7]) > 0): #enough satellites
#                    return(gps_data_to_point(data))
#        except(UnicodeDecodeError):
#            pass

from serial import Serial

uwb_serial = Serial("/dev/ttyUSB0", 115200)

def getDistanceArray(address, size):
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
    sum = 0
    n = 0
    for distance in distances:
        sum += distance
        n += 1
    if (n == 0):
        return 0 #error
    else:
        return (sum / n)

print(getAverage(getDistanceArray("AA:BB", 100)))