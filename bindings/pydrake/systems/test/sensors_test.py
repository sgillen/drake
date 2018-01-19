import numpy as np
import unittest

import pydrake.systems.sensors as m

pt = m.PixelType
pf = m.PixelFormat


class TestSensors(unittest.TestCase):
    def test_image_traits(self):
        t = m.ImageTraits[pt.kRgba8U]
        self.assertEquals(t.kNumChannels, 4)
        self.assertEquals(t.ChannelType, np.uint8)
        self.assertEquals(t.kPixelFormat, pf.Rgba)

        t = m.ImageTraits[pt.kDepth32F]
        self.assertEquals(t.kNumChannels, 1)
        self.assertEquals(t.ChannelType, np.float32)
        self.assertEquals(t.kPixelFormat, pf.kDepth)

        t = m.ImageTraits[pt.kLabel16I]
        self.assertEquals(t.kNumChannels, 1)
        self.assertEquals(t.ChannelType, np.float32)
        self.assertEquals(t.kPixelFormat, pf.kDepth)
