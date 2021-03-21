#!/usr/bin/env python
import math
import random

import rospy
from PIL import Image
from map_localiser.msg import ExtractorLandmark
from map_localiser.msg import ExtractorLandmarks


def rgb_of(name):
    rgb = {
        "mirror": (128, 192, 0)
    }
    return rgb[name] if name in rgb else (255, 255, 255)


NEAREST_SIZE = 7
ERRORIFY = True
error_RGBs = [rgb_of("mirror")]
TRANSFORM = False
tf_quaternion = (1, 0, 0, 0)
ORIGIN_COLOUR = (255, 0, 0)
MARKER_COLOUR = (255, 255, 0)
IMG_WIN_SIZE = (800, 800)


# implement
def transform(quaternion, landmarks):
    return landmarks


def errorify(RGBs, pattern, w, h):
    for landmark in pattern:
        if hash_to_rgb(landmark.hash) in RGBs:
            landmark.x = random.randint(0, h - 1)
            landmark.z = random.randint(0, w - 1)
    return pattern


def distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def rgb_to_hash(RGB):
    return RGB[0] << 16 | RGB[1] << 8 | RGB[2]


def hash_to_rgb(hash):
    r = (hash >> 16) & 255
    g = (hash >> 8) & 255
    b = hash & 255
    return r, g, b


# reads out black pixels and use them as camera positions
def extract(bmpfile):
    im = Image.open(bmpfile)
    pix = im.load()
    w, h = im.size

    origin = (0, 0)

    # find origin
    for x in range(w):
        for y in range(h):
            if pix[x, y] == ORIGIN_COLOUR:
                origin = (x, y)

    nearest = {}
    for x in range(w):
        for y in range(h):
            d = distance(origin[0], origin[1], x, y)
            colour = pix[x, y]
            if colour == ORIGIN_COLOUR:
                continue
            if colour in nearest:
                if d < nearest[colour][0]:
                    nearest[colour] = (d, x, y, colour)
            else:
                nearest[colour] = (d, x, y, colour)
    nearest_set = []
    for colour in nearest:
        nearest_set.append(nearest[colour])
    nearest_set = sorted(nearest_set, key=lambda el: el[0])[:NEAREST_SIZE]
    pattern = []
    pixel_set = []
    for e in nearest_set:
        pattern.append(lm(e[1], 0, e[2], rgb_to_hash(e[3])))
        pixel_set.append((e[1], e[2], e[3]))

    if ERRORIFY:
        pattern = errorify(error_RGBs, pattern, w, h)

    if TRANSFORM:
        pattern = transform(tf_quaternion, pattern)

    for pixel in pixel_set:
        pix[pixel[0], pixel[1]] = MARKER_COLOUR  # display extracted points as yellow
    im.resize(IMG_WIN_SIZE).show()

    return pattern, origin


def publisher():
    rospy.init_node('landmarks_publisher', anonymous=True)
    landmarks_topic = rospy.get_param("/matcher/landmarks_topic", "/landmarks")
    floorplan_file_path = rospy.get_param("/matcher/floorplan_file_path")
    pub = rospy.Publisher(landmarks_topic, ExtractorLandmarks, queue_size=10)

    pattern, origin = extract(floorplan_file_path)

    rate = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        e = ExtractorLandmarks()
        e.header.frame_id = "world"
        e.header.stamp = rospy.Time.now()
        for l in pattern:
            e.landmarks.append(lm(l.x, l.y, l.z, l.hash))
        rospy.loginfo("showing extractions for origin " + str(origin))
        pub.publish(e)
        rate.sleep()


# create landmark
def lm(x, y, z, _hash):
    e = ExtractorLandmark()
    e.x = x
    e.y = y
    e.z = z
    e.hash = _hash
    return e


trial = [lm(1, 1, 1, 4243456), lm(1, 2, 3, 8421504), lm(3, 3, 3, 12599424), lm(0, 0, 0, 8388736)]

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
