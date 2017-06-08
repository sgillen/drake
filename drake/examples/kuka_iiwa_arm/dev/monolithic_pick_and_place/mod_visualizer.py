import numpy as np

from director.drakevisualizer import *

assert(hasattr(DrakeVisualizer, '_addSubscribers'))

class ModVisualizer(DrakeVisualizer):
    name = 'Mod Visualizer'
    def __init__(self, *args, **kwargs):
        # See mainwindowapp.py
        DrakeVisualizer.__init__(self, *args, **kwargs)
        print "Make mod visualizer"

    def _addSubscribers(self):
        # Override drake visualizer's method
        prefix = 'ESTIMATOR_'
        self.subscribers.append(lcmUtils.addSubscriber(prefix + 'DRAKE_VIEWER_LOAD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerLoadRobot))
        self.subscribers.append(lcmUtils.addSubscriber(prefix + 'DRAKE_VIEWER_ADD_ROBOT', lcmrl.viewer_load_robot_t, self.onViewerAddRobot))
        self.subscribers.append(lcmUtils.addSubscriber(prefix + 'DRAKE_VIEWER_DRAW', lcmrl.viewer_draw_t, self.onViewerDraw))
        # self.subscribers.append(lcmUtils.addSubscriber(prefix + 'DRAKE_PLANAR_LIDAR_.*', lcmbot.planar_lidar_t, self.onPlanarLidar, callbackNeedsChannel=True))
        # self.subscribers.append(lcmUtils.addSubscriber(prefix + 'DRAKE_POINTCLOUD_.*', lcmbot.pointcloud_t, self.onPointCloud, callbackNeedsChannel=True))

    def onViewerLoadRobot(self, msg):
        DrakeVisualizer.onViewerLoadRobot(self, msg)
        modifyRobots(self.robots, alpha=0.5, beta=1)

    def injectVisualizerChange(self, obj):
        assert(isinstance(obj, DrakeVisualizer))
        def extraOnViewerLoadRobot(msg):
            # TODO(eric.cousineau): See if newer versions of VTK handle the Z-Buffer better with non-unity alpha with 
            # http://www.vtk.org/Wiki/VTK/Examples/Cxx/Visualization/CorrectlyRenderTranslucentGeometry
            # ... Possibly not...
            modifyRobots(obj.robots, alpha=1, beta=0) # :(
        appendToMethod(obj, 'onViewerLoadRobot', extraOnViewerLoadRobot)
        # Refresh subscribers
        obj._removeSubscribers()
        obj._addSubscribers()

def blendColor(beta, color_a, color_b):
    """
    @param beta 0 - color_a, 1 - color_b
    """
    color_a = np.array(color_a)
    color_b = np.array(color_b)
    return color_a * (1 - beta) + color_b * beta

def modifyRobots(robots, alpha, beta):
    # See DrakeVisualizer.onViewerLoadRobot
    # TODO(eric.cousineau): ONLY change current instance, rather than class def.
    # Go through each link and change its color
    for robot in robots.itervalues():
        for (linkName, link) in robot.iteritems():
            if linkName != "world":
                for geom in link.geometry:
                    polyData = geom.polyDataItem
                    color = polyData.getProperty('Color')
                    color_est = np.array([0, 1, 0])
                    polyData.setProperty('Color', blendColor(beta, color, color_est))
                    polyData.setProperty('Alpha', alpha)

# Can be obj or cls
def appendToMethod(obj, name, extraFunc):
    originalName = "__original__" + name
    if hasattr(obj, originalName):
        # It already has the original
        return
    originalFunc = getattr(obj, name)
    def newFunc(*args, **kwargs):
        originalFunc(*args, **kwargs)
        print "Execute injected portion"
        extraFunc(*args, **kwargs)
    setattr(obj, name, newFunc)
    setattr(obj, originalName, originalFunc)
    print "Injected method: {} - {}".format(name, originalName)
