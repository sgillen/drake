#!/usr/bin/env python

# Copied from:
# https://github.com/kunimatsu-tri/sandbox/blob/09c7a95/lcm/lcm_rgbd_images_subscriber.py
import bot_core
import robotlocomotion
import cv2
import lcm
import numpy as np
import time
import zlib

_VIEW_UPDATE_RATE = 30. # Hz
_BOT_CORE = False

class ImageViewer(object):
    def __init__(self, channel_name):
        self._subscribed_data = None
        self._lcm = lcm.LCM()
        self._lcm.subscribe(channel_name, self.callback)

    def callback(self, channel, data):
        self._subscribed_data = data

    def spin_once(self):
        self._lcm.handle()

    def update_view_with_bot_core_lcmtypes(self):
        images_msg = bot_core.images_t.decode(self._subscribed_data)
        color = images_msg.images[0]
        depth = images_msg.images[1]

        # Color image.
        if color.pixelformat == bot_core.image_t.PIXEL_FORMAT_MJPEG:
            color_data = zlib.decompress(color.data)
        elif color.pixelformat == bot_core.image_t.PIXEL_FORMAT_BGRA:
            color_data = color.data
        else:
            raise Exception("Pixel format {} is not supported.".format(color.pixelformat))

        # Depth image.
        if depth.pixelformat == bot_core.image_t.PIXEL_FORMAT_MJPEG:
            depth_data = zlib.decompress(depth.data)
        elif depth.pixelformat == bot_core.image_t.PIXEL_FORMAT_FLOAT_GREY32:
            depth_data = depth.data
        else:
            raise Exception(
                "Pixel format {} is not supported.".format(depth.pixelformat))

        self.update_view(color_data, depth_data, color.height, color.width,
                         color.row_stride)

    def update_view_with_robotlocomotion_lcmtypes(self):
        image_array_msg = robotlocomotion.image_array_t.decode(self._subscribed_data)
        color = image_array_msg.images[0]
        depth = image_array_msg.images[1]

        # Color image.
        if color.compression_method == robotlocomotion.image_t.COMPRESSION_METHOD_ZLIB:
            color_data = zlib.decompress(color.data)
        elif color.compression_method == robotlocomotion.image_t.COMPRESSION_METHOD_NOT_COMPRESSED:
            color_data = color.data
        else:
            raise Exception("Pixel format {} is not supported.".format(color.compression_method))

        # Depth image.
        if depth.compression_method == robotlocomotion.image_t.COMPRESSION_METHOD_ZLIB:
            depth_data = zlib.decompress(depth.data)
        elif depth.compression_method == robotlocomotion.image_t.COMPRESSION_METHOD_NOT_COMPRESSED:
            depth_data = depth.data
        else:
            raise Exception(
                "Pixel format {} is not supported.".format(depth.compression_method))

        self.update_view(color_data, depth_data, color.height, color.width,
                         color.row_stride)


    def update_view(self, color_data, depth_data, height, width, color_row_stride):
        cv_color_image = np.array(list(color_data))
        cv_color_image = cv_color_image.view(np.uint8)
        cv_color_image = np.reshape(cv_color_image,
                                    (height, width, color_row_stride / width))
        # color_row_stride / width is calculating the number of channels in a pixel.

        cv_depth_image = np.array(list(depth_data))
        # byte array to float array
        cv_depth_image = cv_depth_image.view(dtype=np.float32)
        cv_depth_image = np.reshape(cv_depth_image,
                                    (height, width, 1))
        # 1 is the number of channels in a pixel.
        cv_depth_image = cv_depth_image * 51. # 255 / 5 = 51
        cv_depth_image = np.array(cv_depth_image, np.uint8)

        cv2.imshow("Color Image", cv_color_image)
        cv2.imshow("Depth Image", cv_depth_image)
        cv2.waitKey(2)


def main():
    image_viewer = ImageViewer("DRAKE_RGBD_CAMERA_IMAGES")

    try:
        while True:
            start_time = time.time()
            image_viewer.spin_once()

            if _BOT_CORE:
                image_viewer.update_view_with_bot_core_lcmtypes()
            else:
                image_viewer.update_view_with_robotlocomotion_lcmtypes()

            while time.time() - start_time < 1 / _VIEW_UPDATE_RATE:
                time.sleep(0.005)
                image_viewer.spin_once()

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
