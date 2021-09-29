#!/usr/bin/env python
import math

import time
import pygame
import serial
from OpenGL.GL import *
from OpenGL.GLU import *
from pygame.locals import *

# ser = serial.Serial('/dev/tty.usbserial', 38400, timeout=1)
ser = serial.Serial('COM4', 38400, timeout=1)

ax = ay = az = 0.0
gx = gy = 0.0
yaw_mode = False


def resize(width, height):
    if height == 0:
        height = 1
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45, 1.0 * width / height, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()


def init():
    glShadeModel(GL_SMOOTH)
    glClearColor(0.0, 0.0, 0.0, 0.0)
    glClearDepth(1.0)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST)


def drawText(position, textString):
    font = pygame.font.SysFont("Courier", 18, True)
    textSurface = font.render(textString, True, (255, 255, 255, 255), (0, 0, 0, 255))
    textData = pygame.image.tostring(textSurface, "RGBA", True)
    glRasterPos3d(*position)
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)


def draw():
    global rquad
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity()
    glTranslatef(0, 0.0, -7.0)

    osd_text = "pitch: " + str("{0:.2f}".format(ay)) + ", roll: " + str("{0:.2f}".format(ax))

    if yaw_mode:
        osd_line = osd_text + ", yaw: " + str("{0:.2f}".format(az))
    else:
        osd_line = osd_text

    drawText((-2, -2, 2), osd_line)

    # the way I'm holding the IMU board, X and Y axis are switched 
    # with respect to the OpenGL coordinate system
    if yaw_mode:  # experimental
        glRotatef(az, 0.0, 1.0, 0.0)  # Yaw,   rotate around y-axis
    else:
        glRotatef(0.0, 0.0, 1.0, 0.0)
    glRotatef(ay, 1.0, 0.0, 0.0)  # Pitch, rotate around x-axis
    glRotatef(-1 * ax, 0.0, 0.0, 1.0)  # Roll,  rotate around z-axis

    glBegin(GL_QUADS)
    glColor3f(0.0, 1.0, 0.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(1.0, 0.2, 1.0)

    glColor3f(1.0, 0.5, 0.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(1.0, -0.2, -1.0)

    glColor3f(1.0, 0.0, 0.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)

    glColor3f(1.0, 1.0, 0.0)
    glVertex3f(1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, -1.0)

    glColor3f(0.0, 0.0, 1.0)
    glVertex3f(-1.0, 0.2, 1.0)
    glVertex3f(-1.0, 0.2, -1.0)
    glVertex3f(-1.0, -0.2, -1.0)
    glVertex3f(-1.0, -0.2, 1.0)

    glColor3f(1.0, 0.0, 1.0)
    glVertex3f(1.0, 0.2, -1.0)
    glVertex3f(1.0, 0.2, 1.0)
    glVertex3f(1.0, -0.2, 1.0)
    glVertex3f(1.0, -0.2, -1.0)
    glEnd()


def read_data(FREQ):
    global ax, ay, az, gx, gy

    line_done = 0

    # request data by sending a dot
    ser.write(b".")  # * encode string to bytes
    # while not line_done:
    line = ser.readline()
    data = line.split(b", ")
    if len(data) == 6:
        acc_x = float(data[3])
        acc_y = float(data[4])
        acc_z = float(data[5])
        gyr_x = float(data[0])
        gyr_y = float(data[1])
        gyr_z = float(data[2])

        acy = math.atan2(acc_x, math.sqrt(pow(acc_y, 2) + pow(acc_z, 2))) * 180 / math.pi;
        acx = math.atan2(acc_y, math.sqrt(pow(acc_x, 2) + pow(acc_z, 2))) * 180 / math.pi;


        gx = gx + gyr_x / FREQ
        gy = gy - gyr_y / FREQ

        gx = gx * 0.96 + acx * 0.04;
        gy = gy * 0.96 + acy * 0.04;

        ax = -gx
        ay = -gy



def main():
    global yaw_mode
    FREQ = 30
    video_flags = OPENGL | DOUBLEBUF

    pygame.init()
    screen = pygame.display.set_mode((640, 480), video_flags)
    pygame.display.set_caption("Press Esc to quit, z toggles yaw mode")
    resize(640, 480)
    init()
    frames = 0
    ticks = pygame.time.get_ticks()
    while 1:
        start = time.time()
        event = pygame.event.poll()
        if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
            pygame.quit()  # * quit pygame properly
            break
        if event.type == KEYDOWN and event.key == K_z:
            yaw_mode = not yaw_mode
            ser.write(b"z")
        read_data(FREQ)
        draw()

        pygame.display.flip()
        frames = frames + 1
        stop = time.time()
        FREQ = 1/(stop - start)
        print(FREQ)
    print("fps:  %d" % ((frames * 1000) / (pygame.time.get_ticks() - ticks)))
    ser.close()


if __name__ == '__main__': main()
