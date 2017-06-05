#!/usr/bin/env python

import time

import robotlocomotion as lcmrobotlocomotion
import bot_core as lcmbotcore

from director import lcmUtils
from director import lcmframe
from director import cameraview
from director import framevisualization

# Copying from: spartan:517bffd:apps/iiwa/iiwaManipApp.py
lcm_pose_t = lcmrobotlocomotion.pose_stamped_t

print("""
# Helpers
> tf = transformUtils.frameFromPositionAndRPY([1, 0.5, 2], [0, 30, -135]); vis.updateFrame(tf, 'camera to world', scale=0.5, visible=True)
""")

class HelperApp(object):
    def __init__(self):
        lcmUtils.addSubscriber('DRAKE_RGBD_CAMERA_POSE', lcm_pose_t, self.onCameraPoseMessage, callbackNeedsChannel=True)
        lcmUtils.addSubscriber('DRAKE_DEPTH_SENSOR_POSE', lcm_pose_t, self.onCameraPoseMessage, callbackNeedsChannel=True)

    def setCameraFrame(self, channel, cameraToWorld=None):
        cameraToWorld = cameraToWorld or vtk.vtkTransform()
        vis.updateFrame(cameraToWorld, channel, scale=0.5, visible=True)

    def onCameraPoseMessage(self, cameraToWorldMsg, channel):
        # Fake out position_t
        poseBotCore = convert_pose_to_botcore_position(cameraToWorldMsg)
        cameraToWorld = lcmframe.frameFromPositionMessage(poseBotCore)
        self.setCameraFrame(channel, cameraToWorld)

def convert_pose_to_botcore_position(msg):
    pos_in = lcmbotcore.position_3d_t()
    pos_in.translation = msg.pose.position
    pos_in.rotation = msg.pose.orientation
    return pos_in

# Main Routine
global helperApp
helperApp = HelperApp()
app.restoreDefaultWindowState()
app.initWindowSettings()
