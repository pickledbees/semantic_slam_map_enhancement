#!/usr/bin/env python
import math
import random

import rospy
from PIL import Image
from map_localiser.msg import ExtractorLandmark
from map_localiser.msg import ExtractorLandmarks

rgb = {
    "air": (255, 255, 255),
    "bag": (128, 0, 192),
    "bathtub": (0, 0, 192),
    "bed": (0, 0, 128),
    "bg": (0, 0, 0),
    "blinds": (192, 0, 128),
    "books": (128, 192, 128),
    "bookshelf": (64, 128, 0),
    "box": (192, 64, 128),
    "cabinet": (128, 128, 0),
    "ceiling": (0, 192, 128),
    "chair": (128, 0, 128),
    "clothes": (128, 64, 128),
    "counter": (64, 0, 128),
    "curtain": (0, 64, 0),
    "desk": (64, 128, 128),
    "door": (64, 0, 0),
    "dresser": (128, 64, 0),
    "floor": (0, 128, 0),
    "floormat": (0, 64, 128),
    "fridge": (64, 64, 0),
    "lamp": (128, 128, 64),
    "mirror": (128, 192, 0),
    "nightstand": (0, 0, 64),
    "paper": (64, 192, 0),
    "person": (192, 192, 128),
    "picture": (192, 128, 0),
    "pillow": (0, 192, 0),
    "shelves": (192, 128, 128),
    "shower_c": (64, 64, 128),
    "sink": (0, 128, 64),
    "sofa": (0, 128, 128),
    "table": (128, 128, 128),
    "toilet": (128, 0, 64),
    "towel": (192, 192, 0),
    "tv": (192, 64, 0),
    "wall": (128, 0, 0),
    "whiteboard": (64, 192, 128),
    "window": (192, 0, 0)
}

trivial = map(lambda x: rgb[x], ["bg", "floor", "ceiling", "wall", "window", "person", "air"])


def rgb_of(name):
    return rgb[name] if name in rgb else (255, 255, 255)


NEAREST_SIZE = 6

ERRORIFY = False
ERROR_RGBS = [rgb_of("box"), rgb_of("cabinet"), rgb_of("chair"), rgb_of("clothes"), rgb_of("counter"),
              rgb_of("curtain")][:1]
TRANSFORM = False
TF_QUARTERNION = (1, 0, 0, 0)
NOISE = True
NOISE_SD = 45.0  # noise should match with scale
SCALE = True
SCALE_FACTOR = 100  # scale should match with noise

ORIGIN_COLOUR = (255, 0, 0)
MARKER_COLOUR = (255, 255, 0)
IMG_WIN_SIZE = (800, 800)


def noise(noise_sd, pattern):
    for landmark in pattern:
        landmark.x = landmark.x + random.gauss(0, noise_sd)
        landmark.y = landmark.y + random.gauss(0, noise_sd)
        landmark.z = landmark.z + random.gauss(0, noise_sd)
    return pattern


def scale(scale, pattern):
    for landmark in pattern:
        landmark.x = landmark.x * scale
        landmark.y = landmark.y * scale
        landmark.z = landmark.z * scale
    return pattern


def transform(quaternion, pattern):
    return pattern


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
            if colour == ORIGIN_COLOUR or colour not in list(rgb.values()) or colour in trivial:
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

    if ERRORIFY:
        pattern = errorify(ERROR_RGBS, pattern, w, h)

    for landmark in pattern:
        pixel_set.append((landmark.x, landmark.z, hash_to_rgb(landmark.hash)))

    if TRANSFORM:
        pattern = transform(TF_QUARTERNION, pattern)

    if SCALE:
        pattern = scale(SCALE_FACTOR, pattern)

    if NOISE:
        pattern = noise(NOISE_SD, pattern)

    for pixel in pixel_set:
        pix[pixel[0], pixel[1]] = MARKER_COLOUR
    im.resize(IMG_WIN_SIZE).show()

    return pattern, origin


def publisher():
    rospy.init_node('landmarks_publisher', anonymous=True)
    landmarks_topic = rospy.get_param("/matcher/landmarks_topic", "/landmarks")
    floorplan_file_path = rospy.get_param("/matcher/floorplan_file_path")
    print(floorplan_file_path)
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


if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
