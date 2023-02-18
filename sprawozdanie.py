import matplotlib.pyplot as plt
import json
import os

from ros2_ws.src.gps.gps.ka_utils import Point
from ros2_ws.src.gps.gps.pointsDB import load_points_from_json



# load jsons
jsons = []
for subdir, dirs, files in os.walk("/home/wojtek/pbl/bag-json"):
    for file in files:
        f = open(os.path.join(subdir, file))
        jsons.append(json.load(f))

#transform jsons to files
gps = []
calculated = []
for pomiar in jsons:
    gps_point = Point(pomiar['gps']['x'], pomiar['gps']['y'])
    calculated_point = Point(pomiar['calculated']['x'], pomiar['calculated']['y'])
    if calculated_point.x != 0.0: # xD
        gps.append(gps_point)
        calculated.append(calculated_point)

gps_x = []
gps_y = []
for gps_point in gps:
    gps_x.append(gps_point.x)
    gps_y.append(gps_point.y)

calc_x = []
calc_y = []
for calc_point in calculated:
    calc_x.append(calc_point.x)
    calc_y.append(calc_point.y)

anchors = load_points_from_json()
anchors_x = []
anchors_y = []
for anchor_point in anchors:
    anchors_x.append(anchor_point.x)
    anchors_y.append(anchor_point.y)

#make the plot
plt.plot(gps_y, gps_x, 'ro')
plt.plot(calc_y, calc_x, 'bo')
plt.plot(anchors_y, anchors_x, 'gs')

plt.show()