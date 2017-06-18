#!/usr/bin/env python

import argparse
import numpy as np
import scipy as sp
import scipy.io as spio

import bot_core
import lcm

filename = "/home/eacousineau/proj/tri/proj/dart_impl/_data/sim-logs/okr_resize_rev4.lcmlog"
log = lcm.EventLog(filename, 'r')

body = 'base_link'
channels = {
    'DRAKE_VIEWER_DRAW': {
        'name': 'actual',
        'index': 29,
    },
    'ESTIMATOR_DRAKE_VIEWER_DRAW': {
        'name': 'est',
        'index': 19,
    },
}

class TimeSeries(object):
    def __init__(self):
        self.ts = []
        self.xs = []
    def add(self, t, x):
        self.ts.append(t)
        self.xs.append(x)

data = {
    'actual': TimeSeries(),
    'est': TimeSeries(),
}

last_time = 0.
while last_time < 10.5:
    filepos = log.tell()
    event = log.read_next_event()
    if event is None:
        continue
    info = channels.get(event.channel)
    if info is not None:
        msg = bot_core.viewer_draw_t.decode(event.data)
        t = msg.timestamp / 1.e3
        last_time = max(t, last_time)
        series = data[info['name']]
        pos = msg.position[info['index']]
        quat = msg.quaternion[info['index']]
        print (t, pos)
        series.add(t, (pos, quat))
        print last_time

spio.savemat("./data.mat", data, oned_as='column')
print "Saved"
