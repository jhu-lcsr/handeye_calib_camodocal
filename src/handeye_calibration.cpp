#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <camodocal/calib/HandEyeCalibration.h>
#include <eigen3/Eigen/Geometry>
#include <termios.h>

std::string ARTagTFname, cameraTFname;
std::string EETFname, baseTFname;
tf::TransformListener * listener;
Eigen::Affine3d firstEEInverse, firstCamInverse;
bool firstTransform = true;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > eigenVector;
eigenVector tvecsArm, rvecsArm, tvecsFiducial, rvecsFiducial;

// function getch is from http://answers.ros.org/question/63491/keyboard-key-pressed/
int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering      
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

template<typename Input>
Eigen::Vector3d eigenRotToEigenVector3dAngleAxis(Input eigenQuat){
    Eigen::AngleAxisd ax3d(eigenQuat);
    return ax3d.angle()*ax3d.axis();
}

void addFrame()
{
	ros::Time now = ros::Time::now();
	tf::StampedTransform EETransform, CamTransform; 
	bool hasEE =true, hasCam = true;

	if (listener->waitForTransform(cameraTFname,ARTagTFname,now,ros::Duration(1)))
		listener->lookupTransform(cameraTFname,ARTagTFname,now,CamTransform);
	else
	{
		hasCam = false;
		ROS_WARN("Fail to Cam TF transform between %s to %s", cameraTFname.c_str(), ARTagTFname.c_str());
	}

	if (listener->waitForTransform(baseTFname,EETFname,now,ros::Duration(1)))
		listener->lookupTransform(baseTFname,EETFname,now,EETransform);
	else
	{
		hasEE = false;
		ROS_WARN("Fail to EE TF transform between %s to %s", baseTFname.c_str(), EETFname.c_str());
	}

	if (hasEE and hasEE)
	{
		Eigen::Affine3d eigenEE, eigenCam;
		tf::transformTFToEigen(EETransform, eigenEE);
		tf::transformTFToEigen(CamTransform, eigenCam);
		if (firstTransform)
		{
			firstEEInverse = eigenEE.inverse();
			firstCamInverse = eigenCam.inverse();
			ROS_INFO("Adding first transformation.");
			firstTransform = false;
		}
		else
		{
			Eigen::Affine3d robotTipinFirstTipBase = firstEEInverse * eigenEE;
			Eigen::Affine3d fiducialInFirstFiducialBase = firstCamInverse * eigenCam;

			rvecsArm.push_back(     eigenRotToEigenVector3dAngleAxis(robotTipinFirstTipBase.rotation()        ));
		    tvecsArm.push_back(                                      robotTipinFirstTipBase.translation()     );
		    
		    rvecsFiducial.push_back(eigenRotToEigenVector3dAngleAxis(fiducialInFirstFiducialBase.rotation()   ));
		    tvecsFiducial.push_back(                                 fiducialInFirstFiducialBase.translation());
			ROS_INFO("Hand Eye Calibration Transform Pair Added");

			std::cerr << "EE Relative transform: \n" << robotTipinFirstTipBase.matrix() << std::endl;
			std::cerr << "Cam Relative transform: \n" << fiducialInFirstFiducialBase.matrix() << std::endl;
			Eigen::Vector4d r_tmp = robotTipinFirstTipBase.matrix().col(3); r_tmp[3] = 0;
			Eigen::Vector4d c_tmp = fiducialInFirstFiducialBase.matrix().col(3); c_tmp[3] = 0;
			
			std::cerr << "L2Norm EE: "  << robotTipinFirstTipBase.matrix().block(3,1,0,3).norm() << "vs Cam:" << fiducialInFirstFiducialBase.matrix().block(3,1,0,3).norm()<<std::endl; 
		}
		std::cerr << "EE transform: \n" << eigenEE.matrix() << std::endl;
		std::cerr << "Cam transform: \n" << eigenCam.matrix() << std::endl;
	}
	else
	{
		ROS_WARN("Fail to get one/both of needed TF transform");
	}
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"handeye_calib_camodocal");    
  ros::NodeHandle nh("~");
  //getting TF names
  nh.param("ARTagTF", ARTagTFname,std::string("/camera_2/ar_marker_0"));
  nh.param("cameraTF", cameraTFname,std::string("/camera_2_link"));
  nh.param("EETF", EETFname,std::string("/ee_fixed_link"));
  nh.param("baseTF", baseTFname,std::string("/base_link"));
  nh.param("TransformFile", baseTFname,std::string("TransformFile.yml"));

  ros::Rate r(10); // 10 hz
  listener = new(tf::TransformListener);

  ros::Duration(1.0).sleep(); //cache the TF transforms
  int key = 0;
  ROS_INFO("Press s to add the current frame transformation to the cache.");
  ROS_INFO("Press d to delete last frame transformation.");
  ROS_INFO("Press q to calibrate frame transformation and exit the application.");
  
  while (ros::ok())
  {
    key = getch();
    if ((key == 's') || (key == 'S')){
      std::cerr << "Adding Transform #:" << rvecsArm.size() << "\n";
      addFrame();
      
    }
  	else if ((key == 'd') || (key == 'D'))
  	{
  		ROS_INFO("Deleted last frame transformation. Number of Current Transformation %d",rvecsArm.size());
  		rvecsArm.pop_back();
  		tvecsArm.pop_back();
  		rvecsFiducial.pop_back();
  		tvecsFiducial.pop_back();
  	}
    else if ((key == 'q') || (key == 'Q'))
	{
	  	if (rvecsArm.size() < 5)
	  	{
	  		ROS_WARN("Number of calibration data < 5.");
				ROS_INFO("Node Quit");
	  	}
	  	ROS_INFO("Calculating Calibration...");
	  	camodocal::HandEyeCalibration calib;
	  	Eigen::Matrix4d result;
	  	calib.estimateHandEyeScrew(rvecsArm,tvecsArm,rvecsFiducial,tvecsFiducial,result,false);
	  	std::cerr << "Result: \n" << result << std::endl;
	  	Eigen::Transform<double,3,Eigen::Affine> resultAffine(result);
	  	std::cerr << "Translation: " << resultAffine.translation() << std::endl;
	  	Eigen::Quaternion<double> quaternionResult (resultAffine.rotation());
	  	std::stringstream ss;
	  	ss << quaternionResult.w() << ", " << quaternionResult.x() << ", " << quaternionResult.y() << ", " << quaternionResult.x() << std::endl;
	  	std::cerr << "Rotation: " << ss.str() << std::endl;
	  	
	  	break;
	}
	else
	{
		std::cerr << key << " pressed.\n";
	}
    r.sleep();
  }

  ros::shutdown();
  return (0);
}
