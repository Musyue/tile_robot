#!/usr/bin/env python
# -*- coding: utf-8 -*-
import ros
import cv2
import numpy as np
from collections import defaultdict
def segment_by_angle_kmeans(lines, k=2, **kwargs):
    """Groups lines based on angle with k-means.

    Uses k-means on the coordinates of the angle on the unit circle
    to segment `k` angles inside `lines`.
    """

    # Define criteria = (type, max_iter, epsilon)
    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get('criteria', (default_criteria_type, 10, 1.0))
    flags = kwargs.get('flags', cv2.KMEANS_PP_CENTERS)
    attempts = kwargs.get('attempts', 10)

    # returns angles in [0, pi] in radians
    angles = np.array([line[0][1] for line in lines])
    print "angles",angles
    # multiply the angles by two and find coordinates of that angle
    pts = np.array([[np.cos(2*angle), np.sin(2*angle)]
                    for angle in angles], dtype=np.float32)

    # run kmeans on the coords
    # print pts
    # labels1 = np.zeros(k)
    labels, centers = cv2.kmeans(pts, k,None,criteria, attempts, flags)[1:]
    print "labels",labels,centers
    labels = labels.reshape(-1)  # transpose to row vec
    print "labels", labels
    # segment lines based on their kmeans label
    segmented = defaultdict(list)
    segmented0=defaultdict(list)
    segmented1=defaultdict(list)
    for i, line in zip(range(len(lines)), lines):
        print i,line
        segmented[labels[i]].append(line)
    print "before segmented",segmented
    segmented = list(segmented.values())
    segmented0 = list(segmented[0])
    segmented1 = list(segmented[1])
    print "after segmented\n",segmented
    return segmented,segmented0,segmented1
img = cv2.imread('/data/ros/ur_ws_yue/src/tile_robot/Impedance_control/3.jpg')
gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
blur = cv2.medianBlur(gray, 5)
adapt_type = cv2.ADAPTIVE_THRESH_GAUSSIAN_C
thresh_type = cv2.THRESH_BINARY_INV
bin_img = cv2.adaptiveThreshold(blur, 255, adapt_type, thresh_type, 7, 2)
rho, theta, thresh = 2, np.pi/180, 400
lines = cv2.HoughLines(bin_img, rho, theta, thresh)
# print line
lines1=lines.copy()

def intersection(line1, line2):
    """Finds the intersection of two lines given in Hesse normal form.

    Returns closest integer pixel locations.
    See https://stackoverflow.com/a/383527/5087436
    """
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([
        [np.cos(theta1), np.sin(theta1)],
        [np.cos(theta2), np.sin(theta2)]
    ])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]
def draw_line(segmented,color):
    for ii in xrange(len(segmented)):
        rho, theta = np.array(segmented[ii][0])
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(img, (x1, y1), (x2, y2), color, 6)

def segmented_intersections(lines):
    """Finds the intersections between groups of lines."""

    intersections = []
    for i, group in enumerate(lines[:-1]):
        for next_group in lines[i+1:]:
            for line1 in group:
                for line2 in next_group:
                    intersections.append(intersection(line1, line2))

    return intersections

segmented,segmented0,segmented1 = segment_by_angle_kmeans(lines)
print np.array(segmented)
intersections = segmented_intersections(segmented)
print intersections

draw_line(segmented0,(0,0,255))
draw_line(segmented1,(100,0,255))
for i in intersections:
    cv2.circle(img, (i[0][0],i[0][1]), 5, (255, 0, 0), -1)


cv2.namedWindow('tile_pixel_frame', cv2.WINDOW_NORMAL)
cv2.imshow('tile_pixel_frame', bin_img)
cv2.namedWindow('tile_pixel_frame1', cv2.WINDOW_NORMAL)
cv2.imshow('tile_pixel_frame1', img)
cv2.waitKey(50000)
# cv2.destroyAllWindows()