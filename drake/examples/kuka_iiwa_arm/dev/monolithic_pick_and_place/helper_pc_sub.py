#!/usr/bin/env python

import sys
import os
import time
import robotlocomotion as lcmrobotlocomotion
import bot_core as lcmbotcore
import numpy as np

import lcm

lcm_pointcloud_t = lcmbotcore.pointcloud_t

class HelperSub(object):
    def  __init__(self):
        self.lcm = lcm.LCM()
        self.sub = self.lcm.subscribe('DRAKE_RGBD_POINT_CLOUD', self.callback)

    def run(self):
        try:
            rate = Rate(0.01)
            while True:
                self.lcm.handle()
                rate.sleep()
        except KeyboardInterrupt:
            pass

    def callback(self, channel, data):
        msg = lcm_pointcloud_t.decode(data)
        print "Received"
        points = np.array(msg.points)
        print "{}".format(points.shape)
        # Count all finite points
        is_finite = np.all(np.isfinite(points), axis=1)
        print "non-nan: {}".format(np.count_nonzero(is_finite))
        center = np.mean(points[is_finite], axis=0)
        print center

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
    client = HelperSub()
    client.run()
