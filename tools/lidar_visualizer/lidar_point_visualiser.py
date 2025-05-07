import matplotlib.pyplot as plt
import matplotlib.animation as anim
import serial
import threading
import argparse
import math

parser = argparse.ArgumentParser()
parser.add_argument("-d", "--device", default="/dev/ttyACM0")
parser.add_argument("-b", "--baudrate", default=921600, type=int)
parser.add_argument("-l", "--limit", default=4, type=int)
args = parser.parse_args()

mutex = threading.Lock()

def update(i, ax, list_x, list_y):
    mutex.acquire()
    ax.clear()
    ax.scatter([0], [0], c= 'blue')
    ax.scatter(list_x, list_y, c= 'red')
    lim = args.limit
    ax.set_xlim([-lim,lim])
    ax.set_ylim([-lim,lim])
    mutex.release()

def read_cr(point_list):
    try:
        mutex.acquire()
        for point in point_list.split(">"):
            if not point:
                continue
            x, y = [float(a) for a in point.split(";")]
            list_x.append(x)
            list_y.append(y)
    except Exception:
        pass
    finally:
        mutex.release()


def read_lp(point_list):
    try:
        mutex.acquire()
        for point in point_list.split(">"):
            if not point:
                continue
            a, dist = [float(a) for a in point.split(";")]
            list_x.append(math.cos(a) * dist)
            list_y.append(math.sin(a) * dist)
    except Exception:
        pass
    finally:
        mutex.release()

def read_points_from_serial(list_x, list_y):

    with serial.Serial(args.device, baudrate=args.baudrate) as ser:
        while True:
            try:
                line = ser.read_until(b'\n').decode().replace("\n", "").replace("\r", "")
                type, point_list = line.split("|")
                list_x.clear()
                list_y.clear()
                if type == "LP":
                    read_lp(point_list)
                elif type == "CR":
                    read_cr(point_list)
            except Exception:
                pass

if __name__ == '__main__':
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    list_x = []
    list_y = []
    t = threading.Thread(target=read_points_from_serial, args=(list_x, list_y))
    t.daemon = True
    t.start()
    a = anim.FuncAnimation(fig, update, fargs=(ax, list_x, list_y), repeat=True, interval=200)
    plt.show()
