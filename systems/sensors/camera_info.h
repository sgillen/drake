#pragma once

#include <Eigen/Dense>

#include "drake/common/drake_copyable.h"

namespace drake {
namespace systems {
namespace sensors {

// TODO(kunimatsu-tri) Add camera distortion parameters and other parameters as
// needed.
/**
Simple data structure for camera information that includes the image size
and camera intrinsic parameters.

Please note the following terms:

- frame, pose: denotes an origin and a space-spanning basis in 2D/3D, as in
  @ref multibody_frames_and_bodies. Unless otherwise stated, any described
  frame implies this definition.
- sensor: a device used to take and report measurements.
- image: an array of measurements. Coordinates are in typically in pixels.
- imager: a sensor whose measurements are reported in images.
- camera: an image sensor using lenses and photoreceptive surfaces.
- aperature: a hole through which light travels through.
- image plane: defined by the lens and aperature and captures how the 3D world
  is projected to a 2D image.
- pinhole camera: a physical camera with no lens and a tiny aperature.
- viewing raw: ray from aperature towards scene being captured by the camera
  (orthogonal to the image plane for the pinhole model). In other words, it's
  the way the camera is facing.
- principal point: the intersection of the viewing ray with the image plane,
  typically measured in pixels.

In Drake, all cameras are assumed to be modeled as a pinhole camera (using the
"pinhole model") unless otherwise stated. As in the
[OpenCV documentation](http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html),
the model's image plane will be used rather than pinhole camera's; i.e. it is
presented as in front of the aperature, rather than behind.

Any given camera measures a captured image, which is 2D with pixel coordinates
defined as `(u, v)`. The camera frame is located at frame C which is comprised
of the basis [Cx Cy Cz] and origin point Co, which are defined as follows:

- Co at camera aperature.
- Cz aligned with the viewing ray (orthogonal to the image plane).
- Cx aligned with the `u` axis of the captured image.
- Cy aligned with the `v` axis of the captured image.

This can be summarized as `X-right`, `Y-down`, and `Z-forward` with respect to
the image plane / captured image when visualized in 3D w.r.t. the camera frame.

The projection from coordinates `(X, Y, Z)`, in meters, in the camera frame C
to image coordinates `(u, v)`, in pixels, is defined as:
<pre>
  u = focal_x * (X/Z) + ppx
  v = focal_y * (Y/Z) + ppy
</pre>
where `focal_x, focal_y`, in `pixels / (m / m)`, are the focal lengths and
`ppx, ppy`, in pixels, describe the principal point.

Footnotes:

-  A "video frame" should be considered an image in a video sequence, *not* an
  extension of "frame" as a general term.
*/
class CameraInfo final {
 public:
  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(CameraInfo)

  /**
   Constructor that directly sets the image size, center, and focal lengths.

   @param width The image width in pixels, must be greater than zero.
   @param height The image height in pixels, must be greater than zero.
   @param focal_x The focal length x in pixels.
   @param focal_y The focal length y in pixels.
   @param center_x The x coordinate of the principal point in pixels.
   @param center_y The y coordinate of the principal point in pixels.
  */
  CameraInfo(int width, int height, double focal_x, double focal_y,
             double center_x, double center_y);

  /**
   Constructor that sets the image size and vertical field of view `fov_y`.
   We assume there is no image offset, so the image center `(center_x,`
   `center_y)` is equal to `(width / 2, height / 2)`.  We also assume the
   focal lengths `focal_x` and `focal_y` are identical.  The horizontal
   field of view `fov_x` is calculated using the aspect ratio of the image
   width and height together with the vertical field of view:
   <pre>
     fov_x = 2 * atan(width / height * tan(fov_y / 2)).
   </pre>
   This can be derived from the equations of the focal lengths:
   <pre>
     focal_x = width / 2 / tan(fov_x / 2)
     focal_y = height / 2 / tan(fov_y / 2)
   </pre>
   where `focal_x / focal_y = 1`.

   @param width The image width in pixels, must be greater than zero.
   @param height The image height in pixels, must be greater than zero.
   @param fov_y The vertical field of view.
  */
  CameraInfo(int width, int height, double fov_y);

  /** Returns the width of the image in pixels. */
  int width() const { return width_; }

  /** Returns the height of the image in pixels. */
  int height() const { return height_; }

  /** Returns the focal length x in pixels. */
  double focal_x() const { return intrinsic_matrix_(0, 0); }

  /** Returns the focal length y in pixels. */
  double focal_y() const { return intrinsic_matrix_(1, 1); }

  // TODO(eric.cousineau): Deprecate "center_{x,y}" and use
  // "principal_point_{x,y}" or "pp{x,y}".

  /** Returns the principal point's x coordinate in pixels. */
  double center_x() const { return intrinsic_matrix_(0, 2); }

  /** Returns the principal point's y coordinate in pixels. */
  double center_y() const { return intrinsic_matrix_(1, 2); }

  /** Returns the camera intrinsic matrix. */
  const Eigen::Matrix3d& intrinsic_matrix() const {
    return intrinsic_matrix_;
  }

 private:
  int width_{};
  int height_{};
  // Camera intrinsic parameter matrix. For the detail, see
  // http://docs.opencv.org/2.4/modules/calib3d/doc/
  // camera_calibration_and_3d_reconstruction.html
  Eigen::Matrix3d intrinsic_matrix_;
};

}  // namespace sensors
}  // namespace systems
}  // namespace drake
