from director.lcmlogplayer import LcmLogPlayer, LcmLogPlayerGui
from director import lcmUtils
from director import visualization as vis
import bot_core as lcmbotcore

# Copied from:
# https://github.com/RobotLocomotion/spartan/blob/52b37a6/scripts/sim_playback.py
# TODO(eric.cousineau): See what the root cause is for LCM-GL stuff not working well.

def getDrakeSimTimeForEvent(event):
    if event.channel == 'DRAKE_VIEWER_DRAW':
        msg = lcmbotcore.viewer_draw_t.decode(event.data)
        return msg.timestamp*1000


def initDrakeTimeDisplay():
    def onViewerDraw(msg):
        t = msg.timestamp*1e-3
        vis.updateText('sim time: %.3f' % t, 'sim time')
    lcmUtils.addSubscriber('DRAKE_VIEWER_DRAW', lcmbotcore.viewer_draw_t, onViewerDraw)

print """
NOTE: You may need to hack `lcmlogplayer` to keep checking until it has enough timestamps.
e.g., in {}, change:
    while True:
        # ...
        event = log.read_next_event()
        if not event:
            break
to:
    while lastEventTimestamp < 10.5e6:  # Or whatever the end time is
        # ...
        event = log.read_next_event()
        if not event:
            continue
""".format(director.lcmlogplayer.__file__)

if __name__ == '__main__':

    if len(_argv) != 2:
        raise Exception('usage: %s <filename.lcmlog>' % _argv[0])

    filename = _argv[1]

    logPlayer = LcmLogPlayer()
    logPlayer.playbackFactor = 1
    logPlayer.readLog(filename, eventTimeFunction=getDrakeSimTimeForEvent)
    logPlayer.skipToTime(0.0)

    logPlayerGui = LcmLogPlayerGui(logPlayer)
    logPlayerGui.playbackStartTime = 0.01

    initDrakeTimeDisplay()
