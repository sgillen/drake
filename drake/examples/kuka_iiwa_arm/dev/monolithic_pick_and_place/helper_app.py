#!/usr/bin/env python

import time

import robotlocomotion as lcmrobotlocomotion
import bot_core as lcmbotcore

from director import lcmUtils
from director import lcmframe
from director import cameraview
from director import framevisualization
from director.screengrabberpanel import ScreenGrabberPanel

from mod_visualizer import ModVisualizer

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
modVis = ModVisualizer(view)
modVis.injectVisualizerChange(drakeVisualizer)
app.restoreDefaultWindowState()
app.initWindowSettings()

def getCameraPose():
    c = view.camera()
    print(c.GetPosition(), c.GetFocalPoint(), c.GetRoll())
def setCameraPose():
    c = view.camera()
    # From getCameraPose()
    data = ((1.4161725134734127, 5.238205785468341, 2.650582842731615), (0.38262166129766567, -0.9745965626127705, 0.7470121335901613), -150.1008206678558)
    call = lambda t: t[0](*t[1:])
    map(call, zip((c.SetPosition, c.SetFocalPoint, c.SetRoll), data))
    view.render()

setCameraPose()
recorder = ScreenGrabberPanel.instance
recorder.ui.viewSizeCombo.currentIndex = 2  # "960x720 (4:3)"
recorder.ui.lockViewSizeCheck.click()
recorder.ui.captureRateSpin.value = 60
