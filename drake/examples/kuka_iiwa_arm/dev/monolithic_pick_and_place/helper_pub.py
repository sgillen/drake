#!/usr/bin/env python

import sys
import os
import time
import robotlocomotion as lcmrobotlocomotion
import numpy as np

import lcm

lcm_pose_t = lcmrobotlocomotion.pose_stamped_t

class HelperPub(object):
    def  __init__(self):
        self.lcm = lcm.LCM()

    def run(self):
        try:
            rate = Rate(0.01)
            while True:
                self.publish(rate.elapsed())
                rate.sleep()
        except KeyboardInterrupt:
            pass

    def publish(self, t):
        msg = lcm_pose_t()
        p = msg.pose.position
        freq = 2 * np.pi / 5.
        (p.x, p.y, p.z) = freq * np.sin([t, 2*t, 3*t])
        o = msg.pose.orientation
        (o.w, o.x, o.y, o.z) = freq * np.sin([t, 2*t, 3*t, 4*t])
        self.lcm.publish('DRAKE_RGBD_CAMERA_POSE', msg.encode())

class Rate:
    # Is there something akin to ros.Rate?
    def __init__(self, period):
        self.start = time.time()
        self.last_hit = self.start
        self.period = period

    def sleep(self):
        while time.time() - self.last_hit < self.period:
            time.sleep(self.period / 10.)
        self.last_hit += self.period

    def elapsed(self):
        return time.time() - self.start

if __name__ == "__main__":
    client = HelperPub()
    client.run()
