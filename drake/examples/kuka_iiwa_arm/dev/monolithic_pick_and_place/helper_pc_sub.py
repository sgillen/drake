#!/usr/bin/env python

import sys
import os
import time
import robotlocomotion as lcmrobotlocomotion
import bot_core as lcmbotcore
import numpy as np
from threading import Thread, Lock

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import lcm

lcm_pointcloud_t = lcmbotcore.pointcloud_t

class Plotter:
    def __init__(self):
        self.lock = Lock()
        self.finished = False
        self.action_queue = None

    def init(self):
        self.fig = plt.figure()
        self.ax = plt.subplot(111, projection='3d')
        plt.ion()
        plt.show(block=False)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_zlabel("z")

    def run(self):
        print "Running"
        while not self.finished:
            with self.lock:
                if self.action_queue is not None:
                    self.action_queue()
                    self.action_queue = None
            plt.pause(0.05)

    def plot_points(self, points):
        # Consider copying points if needed
        def action():
            self.ax.scatter(xs=points[:, 0], ys=points[:, 1], zs=points[:, 2])
        with self.lock:
            self.action_queue = action

class HelperSub(object):
    def  __init__(self, plotter):
        self.lcm = lcm.LCM()
        self.sub = self.lcm.subscribe('DRAKE_RGBD_POINT_CLOUD', self.callback)
        self.plotter = plotter
        self.finished = False
        self.thread = Thread(target=self.run)

    def run(self):
        rate = Rate(0.01)
        while not self.finished:
            self.lcm.handle()
            rate.sleep()

    def start(self):
        self.thread.start()

    def join(self):
        self.finished = True
        self.thread.join()

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
        self.plotter.plot_points(points[is_finite])

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
    plotter = Plotter()
    plotter.init()
    client = HelperSub(plotter)
    try:
        client.start()
        plotter.run()    
    except KeyboardInterrupt:
        pass
    client.join()
