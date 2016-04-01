ROS + CamOdoCal Hand Eye Calibration
====================================


This is a ROS node integrating the Hand Eye Calibration implemented in [CamOdoCal](https://github.com/hengli/camodocal). See this [stack exchange question explaining of how Hand Eye Calibration works](http://robotics.stackexchange.com/questions/7163/hand-eye-calibration).

Example uses include determining exact transforms with both positions and orientations of a:

 - camera attached to the floor is relative to a robot arm
 - camera attached to a robot arm tip is relative to a robot arm
 - set of cameras attached to a moving vehicle (this is what camodocal itself implements)


Troubleshooting
---------------

We recommend you collect at least ~36 transforms for a good calibration. If it fails to converge
(i.e. you don't get a good result out). Then you probably have your transforms flipped the wrong
way or there is too much noise in your data to find a sufficiently accurate calibration.
