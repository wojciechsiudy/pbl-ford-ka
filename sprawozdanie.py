import matplotlib.pyplot as plt
import json
import csv
import os
from geopy.distance import geodesic
from ros2_ws.src.gps.gps.ka_utils import Point, calculate_position
from ros2_ws.src.gps.gps.pointsDB import load_points_from_json

## load jsons
#jsons = []
#for subdir, dirs, files in os.walk("/home/wojtek/pbl/bag-json"):
#    for file in files:
#        f = open(os.path.join(subdir, file))
#        jsons.append(json.load(f))
#
##transform jsons to files
#gps = []
#calculated = []
#for pomiar in jsons:
#    gps_point = Point(pomiar['gps']['x'], pomiar['gps']['y'])
#    calculated_point = Point(pomiar['calculated']['x'], pomiar['calculated']['y'])
#    if calculated_point.x != 0.0: # xD
#        gps.append(gps_point)
#        calculated.append(calculated_point)
#
#gps_x = []
#gps_y = []
#for gps_point in gps:
#    gps_x.append(gps_point.x)
#    gps_y.append(gps_point.y)
#
#calc_x = []
#calc_y = []
#for calc_point in calculated:
#    calc_x.append(calc_point.x)
#    calc_y.append(calc_point.y)
#
anchors = load_points_from_json()
anchors_x = []
anchors_y = []
for anchor_point in anchors:
    anchors_x.append(anchor_point.x)
    anchors_y.append(anchor_point.y)

gps_x = []
gps_y = []
calc_x = []
calc_y = []

with open("/home/wojtek/pbl/data.csv", newline="") as file:
    csv_reader = csv.reader(file)
    for row in csv_reader:
        if float(row[0]) != 0.0 and float(row[1]) != 0.0:
            gps_x.append(float(row[0]))
            gps_y.append(float(row[1]))
        anchor_a = Point(0.0, 0.0)
        anchor_b = Point(0.0, 0.0)
        gps_pos = Point(float(row[0]), float(row[1]))
        for item in anchors:
            if item.address == row[2]:
                anchor_a = item
            elif item.address == row[3]:
                anchor_b = item
        pa = (anchor_a.x, anchor_a.y)
        pb = (anchor_b.x, anchor_b.y)
        pg = (gps_pos.x, gps_pos.y)
        calculated = calculate_position(
            anchor_a,
            anchor_b,
            Point(float(row[0]), float(row[1])),
            float(row[4]),
            float(row[5])
        )
        if calculated.x != 0.0 and calculated.y != 0.0:
            calc_x.append(calculated.x)
            calc_y.append(calculated.y)



#make the plot
plt.plot(gps_y, gps_x, 'ro')
plt.plot(calc_y, calc_x, 'bo')
plt.plot(anchors_y, anchors_x, 'gs')

plt.show()