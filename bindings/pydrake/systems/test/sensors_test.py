import numpy as np
import unittest

import pydrake.systems.sensors as m

pt = m.PixelType
pf = m.PixelFormat

pixel_types = [
    pt.kRgba8U,
    pt.kDepth32F,
    pt.kLabel16I,
]

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
        self.assertEquals(t.ChannelType, np.int16)
        self.assertEquals(t.kPixelFormat, pf.kDepth)

    def test_image_shape(self):
        for pixel_type in pixel_types:
            ImageT = m.Image[pixel_type]
            self.assertEquals(ImageT.Traits, m.ImageTraits[pixel_type])

            w = 640
            h = 480
            nc = ImageT.Traits.kNumChannels
            image = ImageT(w, h)
            self.assertEquals(image.width(), w)
            self.assertEquals(image.height(), h)
            self.assertEquals(image.size(), h * w * nc)
            self.assertEquals(image.shape(), (h, w, nc))

            w /= 2
            h /= 2
            image.resize(w, h)
            self.assertEquals(image.shape(), (h, w, nc))

    def test_image_data(self):
        for pixel_type in pixel_types:
            default = 1
            w = 640
            h = 480
            ImageT = m.Image[pixel_type]
            image = ImageT(w, h, default)

            self.assertEquals(image.at(0, 0), default)
            self.assertTrue(np.allclose(image.array(), default))

            image.set(0, 0, 2)
            self.assertEquals(image.at(0, 0), 2)

            arr = image.array()
            marr = image.mutable_array()
            marr[:] = 3
            self.assertTrue(image.at(0, 0), 3)
            self.assertTrue(np.allclose(image.array(), 3))
            self.assertTrue(np.allclose(arr, 3))
