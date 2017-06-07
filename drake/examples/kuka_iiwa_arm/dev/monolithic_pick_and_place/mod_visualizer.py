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
