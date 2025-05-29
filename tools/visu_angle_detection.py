import matplotlib.pyplot as plt
import math

LIDAR_STOP_DISTANCE_MAX = 0.52
LIDAR_STOP_DISTANCE_MIN = 0.186
LIDAR_STOP_ANGLE_START = math.pi / 4
LIDAR_STOP_ANGLE_END = math.pi

def is_obstacled_too_close(angle_dist: float, distance: float) -> bool:
    if distance > LIDAR_STOP_DISTANCE_MAX:
        return False
    ratio = math.sqrt(max(distance - LIDAR_STOP_DISTANCE_MIN, 0.0) / \
            (LIDAR_STOP_DISTANCE_MAX - LIDAR_STOP_DISTANCE_MIN))
    test_angle = ratio * LIDAR_STOP_ANGLE_START + (1.0 - ratio) * LIDAR_STOP_ANGLE_END
    return angle_dist < test_angle / 2


res = 100
list = []
x = []
y = []
for l in range(res):
    dist = LIDAR_STOP_DISTANCE_MIN + (LIDAR_STOP_DISTANCE_MAX - LIDAR_STOP_DISTANCE_MIN) * l / res
    for a in range(360):
        a = (a - 180) / 360 * math.pi
        if not is_obstacled_too_close(abs(a), dist):
            continue
        pt = (math.cos(a) * dist, math.sin(a) * dist)
        list.append(pt)
        x.append(pt[0])
        y.append(pt[1])

plt.scatter(x, y, color = 'hotpink')

plt.show()
