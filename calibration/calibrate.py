#!/usr/bin/env python3
import argparse
import re
import sys
import time
from collections import namedtuple
from math import floor, ceil
from queue import Queue

import pygame
import pygame.font
import evdev
from evdev import ecodes

import logging
import os

log = logging.getLogger("guncon3-calibration")

Postion = namedtuple("Postion", ["x", "y"])

CENTER = 32678


class Guncon3(object):
    def __init__(self, device):
        self.device = device
        self.pos = Postion(0, 0)
        self.pos_n = Postion(0, 0)
        self.calibrated = 0
        self.ax = 0
        self.bx = 0
        self.ay = 0
        self.by = 0
        self.path = device.path
        self.js_num = 0

    @property
    def absinfo(self):
        return [self.device.absinfo(ecodes.ABS_X), self.device.absinfo(ecodes.ABS_Y)]

    @property
    def min_x(self):
        return self.device.absinfo(ecodes.ABS_X).min

    @property
    def max_x(self):
        return self.device.absinfo(ecodes.ABS_X).max

    @property
    def min_y(self):
        return self.device.absinfo(ecodes.ABS_Y).min

    @property
    def max_y(self):
        return self.device.absinfo(ecodes.ABS_Y).max

    @property
    def pos_normalised(self):
        return Postion(self.normalise(self.pos.x, self.min_x, self.max_x),
                       self.normalise(self.pos.y, self.min_y, self.max_y))

    # psakhis: des_normalised number
    @staticmethod
    def desnormalise(self):
        return Postion(int(((self.pos_n.x + CENTER) * (self.max_x - self.min_x) / 65535) + self.min_x),
                       int(((self.pos_n.y + CENTER) * (self.max_y - self.min_y) / 65535) + self.min_y))

    @staticmethod
    def normalise(pos, min_, max_):
        return (pos - min_) / float(max_ - min_)

    @property
    def update(self):
        while True:
            ev = self.device.read_one()
            if ev:
                if ev.type == ecodes.EV_ABS:
                    if ev.code == ecodes.ABS_X:
                        if self.calibrated == 1:
                            self.pos = Postion(ev.value, self.pos.y)
                        else:
                            self.pos = Postion(ev.value, self.pos.y)
                    elif ev.code == ecodes.ABS_Y:
                        if self.calibrated == 1:
                            self.pos = Postion(self.pos.x, ev.value)
                        else:
                            self.pos = Postion(self.pos.x, ev.value)
                if ev.type == ecodes.EV_KEY:
                    yield ev.code, ev.value
            else:
                break

    def set_js(self, js_n):
        self.js_num = js_n

    def calibrate(self, targets, shots, width=320, height=240):
        targets_x = [target[0] for target in targets]
        targets_y = [target[1] for target in targets]
        shots_x = [shot[0] for shot in shots]
        shots_y = [shot[1] for shot in shots]

        s1 = targets_x[0]
        s2 = targets_x[1]
        v1 = min(shots_x)
        v2 = max(shots_x)
        a = (s1 - s2) / (v1 - v2)
        b = (s2 * v1 - s1 * v2) / (v1 - v2)
        x_max = (width - b) / a
        x_min = -b / a
        log.info(f"x s1:{s1} s2:{s2} v1:{v1} v2:{v2} a:{a} b:{b}")
        log.info(f"x_max:{x_max} x_min:{x_min}")

        s1 = targets_y[0]
        s2 = targets_y[2]
        v1 = (shots_y[0] * 1.0 + shots_y[1] * 1.0) / 2
        v2 = (shots_y[2] * 1.0 + shots_y[3] * 1.0) / 2
        a = (s1 - s2) / (v1 - v2)
        b = (s2 * v1 - s1 * v2) / (v1 - v2)
        y_max = (height - b) / a
        y_min = -b / a
        log.info(f"y s1:{s1} s2:{s2} v1:{v1} v2:{v2} a:{a} b:{b}")
        log.info(f"y_max:{y_max} y_min:{y_min}")

        self.calibrated = 1
        # set the X and Y calibration values
        # for evdev
        cmd1 = "evdev-joystick --e {} -a 0 --minimum {} --maximum {} ".format(self.path, int(x_min), int(x_max))
        cmd2 = "evdev-joystick --e {} -a 1 --minimum {} --maximum {} ".format(self.path, int(y_max), int(y_min))
        # for mouse option
        # sacar máximo multiplicador referenciado a la pantalla
        minx = x_min + 0x7FFF
        maxx = x_max + 0x7FFF
        miny = y_min + 32767
        maxy = y_max + 32767

        mx = 0xFFFF / (maxx - minx)  # multiplicador para llevar el rango
        offset_x = minx * mx / 0xFFFF
        offset_x *= -1
        my = 0xFFFF / (maxy - miny)
        offset_y = my * (miny + maxy) / 2
        offset_y /= 0xFFFF
        offset_y *= -1
        p1 = 0.3
        p2 = 0.7
        y1 = min(shots_y)
        y2 = max(shots_y)
        my = (p2 - p1) / ((y2 - y1) / 0xFFFF)
        offset_y = y1 / 0xFFFF * my - 1.7

        cmd_input = "xinput set-prop 'Namco GunCon 3' 'Coordinate Transformation Matrix' {} 0 {} 0 {} {} 0 0 1".format(
            mx, offset_x, my, offset_y)
        log.info("cmd input: {}".format(cmd_input))
        log.info("cmd: {}".format(cmd1))
        log.info("cmd: {}".format(cmd2))
        os.system(cmd1)
        os.system(cmd2)
        os.system(cmd_input)

        # for xinput
        # solve_broken
        a0 = (x_min + x_max) / 2
        b0 = a0
        c0 = 32767.0 / (b0 - x_min)
        d0 = 32767.0 / (x_max - b0)
        p0_0 = int(a0)
        p0_1 = int(b0)
        p0_2 = int(c0 * 16384.0)
        p0_3 = int(d0 * 16384.0)

        a1 = (y_min + y_max) / 2
        b1 = a1
        c1 = 32767.0 / (b1 - y_min)
        d1 = 32767.0 / (y_max - b1)
        p1_0 = int(a1)
        p1_1 = int(b1)
        p1_2 = int(c1 * 16384.0)
        p1_3 = int(d1 * 16384.0)
        if self.js_num == 0:
            path_js = "/dev/input/js0"
        else:
            path_js = "/dev/input/js1"

        jscal_cmd = f"jscal -s 6,1,10,{p0_0},{p0_1},{p0_2},{p0_3},1,10,{p1_0},{p1_1},{p1_2},{p1_3},1,0,129,129,4161663,4260750,1,0,126,126,4260750,4161663,1,0,127,127,4227201,4194176,1,0,126,126,4260750,4161663 {path_js}"
        log.info({jscal_cmd})
        os.system(jscal_cmd)
        # https://gist.github.com/KurtJacobson/37288a0300a9c1b3e859c8dcff403300
        # 1 0 0 0 1 0 0 0 1
        # [hscale] [vskew] [hoffset] [hskew] [vscale] [voffset] 0 0 1
        # xinput set-prop '<device name>' 'Coordinate Transformation Matrix' 1.04 0 -0.02 0 1.04 -0.02 0 0 1

        # self.device.set_absinfo(ecodes.ABS_X, min=int(x_min), max=int(x_max))
        # self.device.set_absinfo(ecodes.ABS_Y, min=int(y_min), max=int(y_max))


WIDTH = 320
HEIGHT = 240
TARGET_SIZE = 20
WHITE = (255, 255, 255)
GREY = (128, 128, 128)

STATE_START = 0
STATE_TARGET = 1
STATE_DONE = 3


def draw_target(size=10):
    image = pygame.Surface((size * 8, size * 8)).convert()
    mid = (size * 8) // 2
    pygame.draw.circle(image, WHITE, (mid, mid), size * 4, 2)

    pygame.draw.line(image, WHITE, (mid, mid - size), (mid, mid + size), 2)
    pygame.draw.line(image, WHITE, (mid - size, mid), (mid + size, mid), 2)

    image.set_colorkey([0, 0, 0])
    return image


def draw_cursor(size=10, color=WHITE):
    image = pygame.Surface((size + 2, size + 2)).convert()
    mid = hsize = size // 2
    pygame.draw.line(image, color, (mid - hsize, mid - hsize), (mid - 2, mid - 2), 2)
    pygame.draw.line(image, color, (mid + hsize, mid - hsize), (mid + 2, mid - 2), 2)
    pygame.draw.line(image, color, (mid - hsize, mid + hsize), (mid - 2, mid + 2), 2)
    pygame.draw.line(image, color, (mid + hsize, mid + hsize), (mid + 2, mid + 2), 2)

    image.set_colorkey([0, 0, 0])
    return image


def blit_center(screen, image, pos):
    screen.blit(image, (pos[0] - (image.get_rect()[2] // 2), pos[1] - (image.get_rect()[3] // 2)), )


def blit_right(screen, image, pos):
    screen.blit(image, (pos[0] - (image.get_rect()[2]), pos[1]))


def calibrar_guncon(guncon, width, height):
    pygame.init()
    pygame.font.init()
    font = pygame.font.Font(None, 20)

    start_text = font.render("Pull the TRIGGER to start calibration", True, WHITE)
    start_text_w = start_text.get_rect()[2] // 2

    pygame.display.set_caption("GunCon 3 two-point calibration")

    screen = pygame.display.set_mode((width, height), pygame.FULLSCREEN)
    clock = pygame.time.Clock()

    state = STATE_START
    running = True
    base_target = height * 30 / 100
    targets = [(base_target, base_target), (width - base_target, base_target),
               (width - base_target, height - base_target), (base_target, height - base_target)]
    target_shots = [(0, 0), (0, 0), (0, 0), (0, 0)]

    cursor = draw_cursor(color=(255, 255, 0))
    target = draw_target()
    onscreen_warning = 0

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_q):
                running = False

        screen.fill((80, 80, 80))

        raw_x, raw_y = guncon.pos
        cx = int((raw_x + 32767) / 65534 * width)
        cy = int((raw_y + 32767) / 65534 * height)
        trigger = False
        for button, value in guncon.update:
            if button == ecodes.BTN_LEFT and value == 1:
                trigger = True
                log.info("Trigger true")
            if button in (ecodes.BTN_MIDDLE, ecodes.BTN_RIGHT) and value == 1:
                running = False
                log.info("BTN_A_B")

        raw_pos_txt = font.render(f"({raw_x}, {raw_y})", True, (128, 128, 255))
        cal_pos_txt = font.render(f"({cx}, {cy})", True, (128, 128, 255))

        screen.blit(raw_pos_txt, (20, height - 40))
        blit_right(screen, cal_pos_txt, (width - 20, height - 40))

        if state == STATE_START:
            screen.blit(start_text, ((width // 2) - start_text_w, height - 60))
            if width > cx >= 0 and height > cy >= 0:  # on screen
                screen.blit(cursor, (cx, cy))
            if trigger:
                state = STATE_TARGET
                target_i = 0
                log.info("Set target at: ({}, {})".format(*targets[target_i]))

        elif state == STATE_TARGET:
            blit_center(screen, target, targets[target_i])
            if trigger:
                target_shots[target_i] = (raw_x, raw_y)
                target_i += 1
                if target_i == len(targets):
                    log.info("Previous tarjet at: ({}, {})".format(*targets[target_i - 1]))
                    log.info("Guncon raw shoted at: ({}, {})".format(*target_shots[target_i - 1]))
                    state = STATE_DONE
                else:
                    log.info("Previous tarjet at: ({}, {})".format(*targets[target_i - 1]))
                    log.info("Guncon raw shoted at: ({}, {})".format(*target_shots[target_i - 1]))
                    log.info("Set target at: ({}, {})".format(*targets[target_i]))

        elif state == STATE_DONE:
            guncon.calibrate(targets, target_shots, width, height)
            state = STATE_START
        fps = font.render(str(round(clock.get_fps())), True, (128, 128, 255))
        screen.blit(fps, (20, 20))

        pygame.display.flip()
        clock.tick(30)


def main():
    def point_type(value):
        m = re.match(r"\(?(\d+)\s*,\s*(\d+)\)?", value)
        if m:
            return int(m.group(1)), int(m.group(2))
        else:
            raise ValueError("{} is an invalid point".format(value))

    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--resolution", default="1366x768")
    parser.add_argument("--center-target", default=(160, 120), type=point_type)
    parser.add_argument("--topleft-target", default=(50, 50), type=point_type)
    parser.add_argument("--capture", default=None)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO)

    try:
        w, h = args.resolution.split("x")
        width, height = int(w), int(h)
    except:
        parser.error("Invalid resolution, eg. 320x240")
        return

    guncon3_dev_1 = None
    guncon3_dev_2 = None
    i = 0
    # find the first guncon3
    for device in [evdev.InputDevice(path) for path in evdev.list_devices()]:
        if device.name == "Namco GunCon 3":
            if i == 0:
                guncon3_dev_1 = device
                log.info("Path: ({})".format(device.path))
                guncon3_dev_1.path = device.path
                i = i + 1
            else:
                guncon3_dev_2 = device
                log.info("Path: ({})".format(device.path))
                i = i + 1
                guncon3_dev_2.path = device.path

    if guncon3_dev_1 is None:
        sys.stderr.write("Failed to find any attached GunCon3 devices")
        return 1

    with guncon3_dev_1.grab_context():

        guncon = Guncon3(guncon3_dev_1)
        guncon.set_js(0)
        calibrar_guncon(guncon, width, height)

    if guncon3_dev_2 is None:
        return

    with guncon3_dev_2.grab_context():
        guncon = Guncon3(guncon3_dev_2)
        guncon.set_js(1)
        calibrar_guncon(guncon, width, height)


if __name__ == "__main__":
    sys.exit(main() or 0)

