import bot_core
import cv2
import lcm
import numpy as np
import time
import zlib

_VIEW_UPDATE_RATE = 30. # Hz

class ImageViewer(object):
    def __init__(self, channel_name):
        self._subscribed_data = None
        self._lcm = lcm.LCM()
        self._lcm.subscribe(channel_name, self.callback)

    def callback(self, channel, data):
        self._subscribed_data = data

    def spin_once(self):
        self._lcm.handle()

    def update_view(self):
        images_msg = bot_core.images_t.decode(self._subscribed_data)
        # print("utime {}".format(images_msg.utime))
        # print("size {} kB".format(images_msg.images[0].size / 1024.))

        # Color image.
        color = images_msg.images[0]
        data = None
        if color.pixelformat == bot_core.image_t.PIXEL_FORMAT_MJPEG:
            data = zlib.decompress(color.data)
        elif color.pixelformat == bot_core.image_t.PIXEL_FORMAT_BGRA:
            data = color.data
        else:
            raise Exception("Pixel format {} is not supported.".format(color.pixelformat))

        cv_color_image = np.array(list(data))
        cv_color_image = cv_color_image.view(np.uint8)
        cv_color_image = np.reshape(cv_color_image,
                                    (color.height, color.width,
                                     color.row_stride / color.width))

        # Depth image.
        depth = images_msg.images[1]
        data = None
        if depth.pixelformat == bot_core.image_t.PIXEL_FORMAT_MJPEG:
            data = zlib.decompress(depth.data)
        elif depth.pixelformat == bot_core.image_t.PIXEL_FORMAT_FLOAT_GREY32:
            data = depth.data
        else:
            raise Exception(
                "Pixel format {} is not supported.".format(depth.pixelformat))

        cv_depth_image = np.array(list(data))
        # byte array to float array
        cv_depth_image = cv_depth_image.view(dtype=np.float32)
        cv_depth_image = np.reshape(cv_depth_image,
                                    (depth.height, depth.width, 1))
        cv_depth_image = cv_depth_image * 51. # 255 / 5 = 51
        cv_depth_image = np.array(cv_depth_image, np.uint8)

        cv2.imshow("Color Image", cv_color_image)
        cv2.imshow("Depth Image", cv_depth_image)
        cv2.waitKey(2)


def main():
    image_viewer = ImageViewer("DRAKE_IMAGE_RGBD")

    try:
        while True:
            start_time = time.time()
            image_viewer.spin_once()
            image_viewer.update_view()
            while time.time() - start_time < 1 / _VIEW_UPDATE_RATE:
                time.sleep(0.01)
                image_viewer.spin_once()

    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
