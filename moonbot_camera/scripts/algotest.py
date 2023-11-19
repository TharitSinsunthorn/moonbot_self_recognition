#!/usr/bin/env python3

import numpy as np 
import math

import matplotlib.pyplot as plt
import matplotlib.patches as patches


def generate_rect(pos, w, h):

    top = pos[1] + h/2
    bottom = pos[1] - h/2 
    left = pos[0] - w/2
    right = pos[0] + w/2

    coor = np.array([[left, top], [right, top], [right, bottom], [left, bottom]])

    centerized = coor[-1]

    return coor

def doOverlap(rect1, rect2):
    if rect1[0][0] > rect2[2][0] or rect2[0][0] > rect1[2][0]:
        return False

    elif rect1[0][1] < rect2[2][1] or rect2[0][1] < rect1[2][1]:
        return False

    else:
        return True

def plotrect(rect1, rect2):
    # Create a figure and axis
    fig, ax = plt.subplots()

    # Create a rectangle patch
    rect1 = plt.Rectangle(rect1[-1], (rect1[1][0] - rect1[3][0]), (rect1[0][1] - rect1[2][1]), edgecolor='r', facecolor='none')
    rect2 = plt.Rectangle(rect2[-1], (rect2[1][0] - rect2[3][0]), (rect2[0][1] - rect2[2][1]), edgecolor='b', facecolor='none')
    # Add the rectangle patch to the axis
    ax.add_patch(rect1)
    ax.add_patch(rect2)

    # Set axis limits for better visualization
    ax.set_xlim([-6, 6])
    ax.set_ylim([-6, 6])
    ax.grid(True)

    # Set labels and title (optional)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.title('Rectangle Plot')

    # Display the plot
    plt.show()


def main():

    rect1 = generate_rect((0,0), 2, 3)
    print(rect1)
    rect2 = generate_rect((1.5,0), 2, 3)

    print(doOverlap(rect1, rect2))

    plotrect(rect1, rect2)


if __name__ == "__main__":
    main()
