from ka_utils import Point, get_position

position = Point(21.3223324, 12.3242342)

points = [Point(34.343224, 42.342424, "AA:BB"), Point(53.3432423, 23.453545445, "BB:22")]

while (True):
    position = get_position()
    print(position.x, position.y)
    for point in points:
        if position.is_around(point):
            if (point.is_observed):
                pass
            else:
                pass #delegate thread to read distance, AB:CD400 - adres AB:CD 400 pakietów - po 10 nieudanych olać