#!/usr/bin/env python

import argparse
import numpy as np

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

last_time = 0.
while last_time < 10.5:
    filepos = log.tell()
    event = log.read_next_event()
    if event is None:
        continue
    info = channels.get(event.channel)
    if info is not None:
        msg = bot_core.viewer_draw_t.decode(event.data)
        last_time = msg.timestamp / 1.e3
        print last_time
