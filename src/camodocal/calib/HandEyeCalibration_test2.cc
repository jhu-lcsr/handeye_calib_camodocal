#include "HandEyeCalibration.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>

using namespace Eigen;

#define DEBUG(msg) std::cout << msg << std::endl;


/*
-------------------------------------------------------------

-------------------------------------------------------------
*/

template <typename Input>
static Eigen::Vector3d
ToEulerAngle(Input eigenQuat)
{
    Eigen::AngleAxisd ax3d(eigenQuat);
    return ax3d.angle() * ax3d.axis();
}

static Isometry3d
MakeRandomT()
{
    Isometry3d t;
    Quaterniond q = Quaterniond::UnitRandom();
    t = q;
    t.translation() = Vector3d::Random();
    return t; 
}

typedef std::vector< Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> >  Eigen3dVec;

int
main(int argc, char *argv[]) {
    srand((unsigned int) time(0));
    
    Isometry3d X = MakeRandomT();
    std::vector<Isometry3d> As(5), Bs(5);
    for (size_t i = 0; i < As.size(); ++i)
        As[i] = MakeRandomT();

    Isometry3d Xinv = X.inverse();
    for (size_t i = 0; i < As.size(); ++i)
        Bs[i] = Xinv * As[i] * X;

    // camodocal::HandEyeCalibration calib;
    Eigen3dVec Ras, Tas, Rbs, Tbs;
    
    for (size_t i = 0; i < As.size(); ++i) {
        Ras.push_back(ToEulerAngle(As[i].rotation()));
        Tas.push_back(As[i].translation());
        Rbs.push_back(ToEulerAngle(Bs[i].rotation()));
        Tbs.push_back(Bs[i].translation());        
    }

    std::cout << X.matrix() << std::endl;
    std::cout << Ras[0] << std::endl;
    std::cout << Rbs[0] << std::endl;
    std::cout << Tas[0] << std::endl;
    
    Eigen::Matrix4d X_estimated; 
    camodocal::HandEyeCalibration::estimateHandEyeScrew(Ras, Tas, Rbs, Tbs, X_estimated);
    std::cout << X_estimated - X.matrix() << std::endl;
    return 0;
}

