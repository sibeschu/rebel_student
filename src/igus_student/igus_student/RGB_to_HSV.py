#!/usr/bin/env python3

import cv2
import numpy as np

# format: ("label", (R, G, B))
colors = [
    ("red puck",  (99, 22, 18)),
    ("red puck",  (101, 25, 21)),
    ("red puck",  (117, 28, 28)),
    ("red puck",  (112, 43, 45)),
    ("blue puck", (12, 41, 86)),
    ("blue puck", (22, 49, 87)),
    ("floor",     (29, 63, 78)),
    ("floor",     (24, 57, 73)),
]

for label, rgb in colors:
    bgr = [[list(rgb)[::-1]]]
    hsv = cv2.cvtColor(np.uint8(bgr), cv2.COLOR_BGR2HSV)[0][0]
    print(f"{label:10s} RGB {rgb} -> OpenCV HSV {tuple(hsv)}")
