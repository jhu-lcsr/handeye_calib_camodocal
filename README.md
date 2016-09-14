ROS + CamOdoCal Hand Eye Calibration
====================================

This is a ROS node integrating the Hand Eye Calibration implemented in [CamOdoCal](https://github.com/hengli/camodocal). See this [stack exchange question explaining of how Hand Eye Calibration works](http://robotics.stackexchange.com/questions/7163/hand-eye-calibration).

Example uses include determining exact transforms with both positions and orientations of a:

 - camera attached to the floor is relative to a robot arm
 - camera attached to a robot arm tip is relative to a robot arm
 - set of cameras attached to a moving vehicle (this is what camodocal itself implements)
 - two robot arms bolted together

When using this with a robot arm, move it around to a variety of poses and orientations, make sure any data sources that lag behind settle down, then record each pair of poses between the robot base and the robot tip, and between the eye/camera base and the marker, fiducial, or AR tag it is viewing.

This will save out a yaml file with the results. Be sure to load the results into your system using the data formatted as a rotation matrix, dual quaternion, or quaternion + translation. Roll Pitch Yaw can degenerate and will often be innacurate! 

Troubleshooting
---------------

We recommend you collect at least ~36 accurate transforms for a good calibration. If it fails to 
converge (i.e. you don't get a good result out). Then you probably have your transforms flipped 
the wrong way or there is too much noise in your data to find a sufficiently accurate calibration.

### Examples of "too much noise" when taking data

#### Your cameras must be calibrated

Camera calibration is  very important! If they aren't calibrated then the poses being fed into the algorithm will be innacurate, won't correspond, and thus the algorithm won't be able to find even a decent approximate solution and will just exit, printing an error.

#### Your robot and cameras must be rigidly fixed

Hand eye calibration solves for a rigid body transform, so if the whole system isn't rigidly fixed the transform you are solving for is constantly changing and thus impossible to find accurately. For example, if you have a camera and a fixed robot base, check that your robot is securely bolted to a surface. Tighten those bolts up! Also ensure the camera is securely and rigidly fixed in place in a similar fasion. Check for any wobbling and make sure to wait for everything to become still before taking your data points. 


Authors
-------

Andrew Hundt <ATHundt@gmail.com>
Felix Jonathan <fjonath1@jhu.edu>

Acknowledgements
----------------

[Hand-Eye Calibration Using Dual Quaternions](https://www.informatik.uni-kiel.de/inf/Sommer/doc/Publications/kd/ijrr99.pdf)

    @article{daniilidis1999hand,
      title={Hand-eye calibration using dual quaternions},
      author={Daniilidis, Konstantinos},
      journal={The International Journal of Robotics Research},
      volume={18},
      number={3},
      pages={286--298},
      year={1999},
      publisher={SAGE Publications}
    }


[CamOdoCal](https://github.com/hengli/camodocal)

    Lionel Heng, Bo Li, and Marc Pollefeys,
    CamOdoCal: Automatic Intrinsic and Extrinsic Calibration of a Rig with Multiple Generic Cameras and Odometry,
    In Proc. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.

    Lionel Heng, Mathias Bürki, Gim Hee Lee, Paul Furgale, Roland Siegwart, and Marc Pollefeys,
    Infrastructure-Based Calibration of a Multi-Camera Rig,
    In Proc. IEEE International Conference on Robotics and Automation (ICRA), 2014.

    Lionel Heng, Paul Furgale, and Marc Pollefeys,
    Leveraging Image-based Localization for Infrastructure-based Calibration of a Multi-camera Rig,
    Journal of Field Robotics (JFR), 2015.
