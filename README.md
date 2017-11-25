ROS + CamOdoCal Hand Eye Calibration
====================================

This is a ROS node integrating the Hand Eye Calibration implemented in [CamOdoCal](https://github.com/hengli/camodocal). See this [stack exchange question explaining how Hand Eye Calibration works](http://robotics.stackexchange.com/questions/7163/hand-eye-calibration).

Example uses include determining exact transforms with both positions and orientations of a:

 - camera attached to the floor is relative to a robot arm
 - camera attached to a robot arm tip relative to a robot arm
 - set of cameras attached to a moving vehicle (this is what camodocal itself implements)
 - two robot arms bolted together

[keynote presentation explaining many details about hand eye calibration](https://www.icloud.com/keynote/AwBUCAESEJ6BPEHy1-J_58iY9QpDxmMaKWYQ8JT4T8wVxHSfiNn7vMJH1IuI3bnUaJeS8H0P8z768Qw95BLoFg2qMCUCAQEEILObYmsh9SaWe-3YhL6v1kNVeciwzjsktabBmlhsO661#Optimal_Hand_Eye_Calibration) for those that are interested. Practical code and instructions to calibrate your robot can be found below.

![Hand Eye Calibration Basics][1]

![Two Common Solutions to Hand Eye Calibration][2]

![AX=XB Hand Eye Calibration Solution][3]

![AX=ZB Hand Eye Calibration Solution][4]

Feeding data into CamOdoCal
---------------------------

 1. Each measurement taken at a different time, position, and orientation narrows down the possible transforms that can represent the unknown X

 2. Record a list of many transforms A and B taken between different time steps, or relative to the first time step
      - Rotations are in AxisAngle = UnitAxis*Angle format, or [x_axis,y_axis,z_axis]*𝜃_angle 
         - ||UnitAxis||=1
         - || AxisAngle || = 𝜃_angle
      - Translations are in the normal [x,y,z] format
 3. Pass both vectors into EstimateHandEyeScrew()
 4. Returns X in the form of a 4x4 transform estimate

![Camodocal Hand Eye Calibration Details][5]

  [1]: http://i.stack.imgur.com/7k4D3.jpg
  [2]: http://i.stack.imgur.com/d4nVb.jpg
  [3]: http://i.stack.imgur.com/wdOyg.jpg
  [4]: http://i.stack.imgur.com/zRQ1i.jpg
  [5]: http://i.stack.imgur.com/Cvc75.jpg


When using this with a robot arm, move it around to a variety of poses and orientations, make sure any data sources that lag behind settle down, then record each pair of poses between the robot base and the robot tip, and between the eye/camera base and the marker, fiducial, or AR tag it is viewing.

This will save out a yaml file with the results. Be sure to load the results into your system using the data formatted as a rotation matrix, dual quaternion, or quaternion + translation. Roll Pitch Yaw can degenerate and will often be inaccurate!

Installation
------------

### Linux
All dependencies can be installed via scripts in the [robotics_setup](https://github.com/ahundt/robotics_setup) repository on `Ubuntu 14.04` or `Ubuntu 16.04`.

### MacOS

On OS X you can use [homebrew](http://brew.sh) and the [homebrew-robotics](https://github.com/ahundt/homebrew-robotics) tap to install all dependencies.

### ROS (both Linux + MacOS)

Once you've completed the Linux or MacOS steps, follow normal ros source package installation procedures with catkin build.

### Dependencies

If installing manually, be sure to follow the instructions for each library as there are specific steps required depending on your OS.

- [ROS indigo or kinetic](ros.org)
- [OpenCV 2 or 3](opencv.org) with (recommended) nonfree components
    - Note handeye_calib_camodocal does not call any nonfree components, but some users have had difficulty configuring CMake to compile and install all the other dependencies without them.
    - OpenCV3 puts their nonfree components in [opencv-contrib](https://github.com/opencv/opencv_contrib).
- [Eigen3](eigen.tuxfamily.org)
- [ceres-solver](ceres-solver.org)
- [glog](https://github.com/google/glog)
  - If you encounter an error about `providing "FindGlog.cmake" in CMAKE_MODULE_PATH`, try installing glog from source.
- [gflags](https://github.com/gflags/gflags)

Examples
--------

There are example pre-recorded transforms in the `example` folder, all config files are expected to be in `handeye_calib_camodocal/launch` folder by default, but if that doesn't work try checking the `~/.ros/` folder.

- [launch/handeye_example.launch](launch/handeye_example.launch)
    - This configures the files transforms are loaded from and saved to, as well as rostopics if reading live data.
- [example/TransformPairsOutput.yml](example/TransformPairsOutput.yml)
    - This contains the set of transforms you record with the launch script, which are input into the solver.
- [example/CalibratedTransform.yml](example/CalibratedTransform.yml)
    - This transform is your final results found by the solver.

To verify that the software is working run:

    roslaunch handeye_calib_camodocal handeye_example.launch

You should see output like the following:

```
# INFO: Before refinement: H_12 =
  -0.962926   -0.156063     0.22004 -0.00802514
  -0.176531    0.981315  -0.0765322   0.0242905
  -0.203985   -0.112539   -0.972484   0.0550756
          0           0           0           1
Ceres Solver Report: Iterations: 89, Initial cost: 1.367791e+01, Final cost: 6.005694e-04, Termination: CONVERGENCE
# INFO: After refinement: H_12 =
  -0.980558    0.184959   0.0655414  0.00771561
  0.0495028  -0.0900424    0.994707   0.0836796
   0.189881    0.978613   0.0791359 -0.00867321
          0           0           0           1
Result from /ee_link to /ar_marker_0:
  -0.980558    0.184959   0.0655414  0.00771561
  0.0495028  -0.0900424    0.994707   0.0836796
   0.189881    0.978613   0.0791359 -0.00867321
          0           0           0           1
Translation (x,y,z) :  0.00771561   0.0836796 -0.00867321
Rotation (w,x,y,z): -0.046193, 0.0871038, 0.672938, 0.733099

Result from /ar_marker_0 to /ee_link:
  -0.980558    0.184959   0.0655414  0.00771561
  0.0495028  -0.0900424    0.994707   0.0836796
   0.189881    0.978613   0.0791359 -0.00867321
          0           0           0           1
Inverted translation (x,y,z) : 0.00507012 0.0145954 -0.083056
Inverted rotation (w,x,y,z): -0.046193, 0.0871038, 0.672938, 0.733099
0.046193 0.0871038 0.672938 0.733099

Writing calibration to "/home/cpaxton/catkin_ws/src/handeye_calib_camodocal/launch/CalibratedTransform.yml"...
[handeye_calib_camodocal-1] process has finished cleanly
log file: /home/cpaxton/.ros/log/a829db0a-f96b-11e6-b1dd-fc4dd43dd90b/handeye_calib_camodocal-1*.log
all processes on machine have died, roslaunch will exit
shutting down processing monitor...
... shutting down processing monitor complete
done
```

The full terminal session can be found at:

 - [example/terminal_session.txt](example/terminal_session.txt)

Recording your own Transforms
-----------------------------

To record your own session, modify [launch/handeye_file.launch](launch/handeye_file.launch) to specify the ROS topics that will publish the poses between which you wish to calibrate, then run:


    roslaunch handeye_calib_camodocal handeye_file.launch

If you have difficulty we cover just about every problem we've seen below in the troubleshooting section. It can also help to see this [stack exchange question explaining how Hand Eye Calibration works](http://robotics.stackexchange.com/questions/7163/hand-eye-calibration)

After you run, be sure to back up `TransformPairsInput.yml` and `CalibratedTransform.yml` so you don't lose all
the transforms and positions you saved!

#### How do I get transforms between the camera and an object it sees?

If a camera is calibrated it is possible to estimate the transform from the camera to a printed pattern with known dimensions. I don’t recommend using a checkerboard for hand eye calibration because the pattern is ambiguous. Use something like:

 - artoolkit.org
 - https://github.com/ros-perception/ar_track_alvar
 
 They provide instructions on how to set up your camera and create patterns that can be used to generate transforms.

Troubleshooting
---------------

#### Saved Files Not Loading?

If you have trouble finding the saved files, the default working directory of ROS applications is in the `~/.ros/` folder, so try looking there. Be sure to also check your launch file which is typically
[launch/handeye_file.launch](launch/handeye_file.launch) this determines if transforms will be loaded
from a running robot or saved files, as well as where save files are placed.

#### Collecting Enough Data

We recommend you collect at least ~36 accurate transforms for a good calibration. If it fails to
converge (i.e. you don't get a good result out), then you probably have your transforms flipped
the wrong way or there is too much noise in your data to find a sufficiently accurate calibration.

### Eliminating Sensor Noise

One simple method to help deal with this problem is to create a new node that reads the data you want
to read and save a rolling average of the pose. This helps to stabilize the results. There are better
methods such as a kalman filter that could handle this even better. If you take a rolling average,
make sure each time you take the data the robot has been set in a single position for the entire duration
of the time for which the rolling average is being taken, because any error here will throw off the results.


### Examples of "too much noise" when taking data

If there is too much noise you will probably see the following error:

```
normalization could not be handled. Your rotations and translations are probably either not aligned or not passed in properly
```

That means there is probably too much variation in the data you are reading to get an accurate solution.
For example, if you watch the pose of an AR tag and it wobbles a little or flips this will prevent an
accurate solution from being found. One way to help this is to ensure the system is completely stationary and then interpolate (average) the poses across several frames, again ensuring the system is completely stationary before recording the frame and then finally moving to the next position and repeating the process.

#### Your cameras must be calibrated

Camera calibration is  very important! If they aren't calibrated then the poses being fed into the algorithm will be inaccurate, won't correspond, and thus the algorithm won't be able to find even a decent approximate solution and will just exit, printing an error.

#### Your robot and cameras must be rigidly fixed

Hand eye calibration solves for a rigid body transform, so if the whole system isn't rigidly fixed the transform you are solving for is constantly changing and thus impossible to find accurately. For example, if you have a camera and a fixed robot base, check that your robot is securely bolted to a surface. Tighten those bolts up! Also ensure the camera is securely and rigidly fixed in place in a similar fasion. Check for any wobbling and make sure to wait for everything to become still before taking your data points.

#### Sanity Check by Physically Measuring

Slight distortion or variation in time stamp while the arm moves slightly as you hold it can still throw it off. One additional way to test that is to have the arm go to two distant positions, and the length of the change in checkerboard poses should be equal to the length of the change in end effector tip poses assuming you can keep the orientation constant.

#### Sanity Check via Simulation

If you’re concerned it is a bug in the algorithm you can run it in simulation with v-rep or gazebo (os + v-rep python script is in the repo) to verify it works, since that will avoid all physical measurement problems. From there you could consider taking more real data and incorporating the real data to narrow down the source of the problem.

#### Sanity Check Transforms and when loading from files

If you're loading from a file you've modified by hand, check if your matrices are transposed, inverted, or in very unusual cases even just the 3x3 Rotation component of the 4x4 rotation matrix may be transposed.

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

```
<node pkg="tf" type="static_transform_publisher" name="endpoint_to_marker" args=" -0.00746998     0.101617 -0.000671928  0.513209 0.492549 0.513209  -0.48498   $(arg ee_frame) /endpoint_marker 10"/>
```

Questions? Here is what we need to know.
----------------------------------------

If you try running this and have a question please create a diagram of your use case so we can understand how you are setting up the equations, then create a [github issue](https://github.com/jhu-lcsr/handeye_calib_camodocal/issues).

See this [stack exchange question explaining of how Hand Eye Calibration works](http://robotics.stackexchange.com/questions/7163/hand-eye-calibration) for an example of such a diagram.

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
    
  
References
----------

- Strobl, K., & Hirzinger, G. (2006) . Optimal hand-eye calibration. In 2006 IEEE/RSJ international conference on intelligent robots and systems (pp. 4647–4653), October 2006.
- [Technical University of Munich (TUM) CAMP lab wiki ](http://campar.in.tum.de/Chair/HandEyeCalibration)
- K. Daniilidis, “Hand–Eye Calibration Using Dual Quaternions,” Int. Journal of Robs. Research, vol. 18, no. 3, pp. 286–298, June 1999.
- E. Bayro–Corrochano, K. Daniilidis, and G. Sommer, “Motor–Algebra for 3D Kinematics: The Case of Hand–Eye Calibration,” Journal for Mathem. Imaging and Vision, vol. 13, no. 2, pp. 79–100, Oct. 2000. 
- F. Dornaika and R. Horaud, “Simultaneous Robot–World and Hand– Eye Calibration,” IEEE Trans. on Robs. and Aut., vol. 14, no. 4, pp. 617–622, August 1998. 
- Note: figures and text are from mixed sources including the presentation author, the various papers referenced, and the TUM wiki.
