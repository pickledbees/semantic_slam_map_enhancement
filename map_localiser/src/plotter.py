#!/usr/bin/env python
import math

import rospy
from map_localiser.msg import MatchResult
from matplotlib import pyplot as plt
from numpy import corrcoef as corr


# script to access the plots


def distanceLandmark(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.z - b.z) ** 2)


def distanceFloorplan(a, b):
    return math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2)


def plot(chain, pattern):
    # form observations
    Dm = []
    Df = []
    l = len(chain)
    for i in range(1, l):
        for j in range(i):
            Dm.append(distanceLandmark(pattern[i], pattern[j]))
            Df.append(distanceFloorplan(chain[i], chain[j]))
    plt.clf()
    plt.xlabel("Octomap distances")
    plt.ylabel("Floorplan distances")
    plt.scatter(Dm, Df)
    fig = plt.figure(1)
    fig.text(.5, 0.02, "corr=" + str(corr(Dm, Df)[0][1]), ha="center")
    fig.set_size_inches(7, 8, forward=True)

    plt.draw()
    plt.pause(0.00000000001)


def callback(data):
    if len(data.chains) == 0:
        rospy.loginfo("no chain to plot")
    else:
        plot(data.chains[-1].elements, data.pattern)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plotter', anonymous=True)

    rospy.Subscriber("/match_results", MatchResult, callback)

    # spin() simply keeps python from exiting until this node is stopped
    plt.show()
    rospy.spin()


if __name__ == '__main__':
    listener()
