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

If you have trouble finding the saved files, the default working directory of ROS applications is in the `~/.ros/` folder, so try looking there.

We recommend you collect at least ~36 accurate transforms for a good calibration. If it fails to 
converge (i.e. you don't get a good result out). Then you probably have your transforms flipped 
the wrong way or there is too much noise in your data to find a sufficiently accurate calibration.


### Examples of "too much noise" when taking data

#### Your cameras must be calibrated

Camera calibration is  very important! If they aren't calibrated then the poses being fed into the algorithm will be innacurate, won't correspond, and thus the algorithm won't be able to find even a decent approximate solution and will just exit, printing an error.

#### Your robot and cameras must be rigidly fixed

Hand eye calibration solves for a rigid body transform, so if the whole system isn't rigidly fixed the transform you are solving for is constantly changing and thus impossible to find accurately. For example, if you have a camera and a fixed robot base, check that your robot is securely bolted to a surface. Tighten those bolts up! Also ensure the camera is securely and rigidly fixed in place in a similar fasion. Check for any wobbling and make sure to wait for everything to become still before taking your data points. 

Example output
--------------

Here is an example output of what you should expect when a run is executed successfully:

```
Writing pairs to "/home/cpaxton/catkin_ws/src/handeye_calib_camodocal/launch/TransformPairsInput.yml"...
q[ INFO] [1473813682.393291696]: Calculating Calibration...
# INFO: Before refinement: H_12 = 
-0.00160534     0.99916   0.0409473 -0.00813108
-0.00487176  -0.0409546    0.999149     0.10692
  0.999987  0.00140449  0.00493341   0.0155885
         0           0           0           1
Ceres Solver Report: Iterations: 99, Initial cost: 1.882582e-05, Final cost: 1.607494e-05, Termination: CONVERGENCE
# INFO: After refinement: H_12 = 
-0.00282176     0.999009    0.0444162  -0.00746998
  0.0121142   -0.0443789     0.998941     0.101617
   0.999923   0.00335684    -0.011977 -0.000671928
          0            0            0            1
Result: 
-0.00282176     0.999009    0.0444162  -0.00746998
  0.0121142   -0.0443789     0.998941     0.101617
   0.999923   0.00335684    -0.011977 -0.000671928
          0            0            0            1
Translation:  -0.00746998     0.101617 -0.000671928
Rotation: -0.48498 0.513209 0.492549 0.513209
```

Note that this run is not a perfect one with errors of 5 mm over a motion of 1 m.

#### Cost

One key piece of information is the output of the cost function, which is a metric representing an estimate of solution accuracy:

```
Initial cost: 1.882582e-05, Final cost: 1.607494e-05
```

With a really good run where the calibration is dead on the final cost should be on the order of 1e-13 or 1e-14.

#### Results

Now lets take a look at the results:

```
Translation:  -0.00746998     0.101617 -0.000671928
Rotation: -0.48498 0.513209 0.492549 0.513209
```

The translation is in xyz format, and the rotation is in quaternion format. It is important to note that this tool and camodocal use the eigen Quaternion format which orders the four values stored in a quaternion wxyz. ROS launch files, by comparison store the data in the order xyzw. That means when copying the results into ROS you must move the first entry of the rotation to the end.

Here is an example of all 7 numbers from above correctly put into a ros launch file:

<node pkg="tf" type="static_transform_publisher" name="endpoint_to_marker" args=" -0.00746998     0.101617 -0.000671928  0.513209 0.492549 0.513209  -0.48498   $(arg ee_frame) /endpoint_marker 10"/>


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
