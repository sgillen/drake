#!/usr/bin/env python
from director import ioUtils
from director import transformUtils

# vis = globals()['vis']

p_meas = ioUtils.readPolyData('blue_funnel_meas.vtp')
p_model = ioUtils.readPolyData('blue_funnel.stl')

vis_meas = vis.showPolyData(p_meas, 'meas')
vis_model = vis.showPolyData(p_model, 'model')

T_WM = transformUtils.transformFromPose(
    [-0.0905826680449818, -0.7170680310741089, 1.0328316719986117],
    [-0.4511915145490044, 0.8915555129070134, -0.03518433604141043, -0.017805816505091585])
f_model = vis_model.getChildFrame()
# How to do this properly?
f_model.transform = T_WM
