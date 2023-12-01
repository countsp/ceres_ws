#include <iostream>
#include <string>
#include <vector>

#include <algorithm>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "glog/logging.h"

using namespace std;
using namespace Eigen;

void test_uint()
{
cout<<endl;
const uint64_t kMagic = 0x7b1d1f7b5bf501db;
uint64_t size = kMagic;
for(int i =0;i<8;i++)
	{
		cout<<(size & 0xff) <<",";
		size>>=8;
		cout<< size <<endl;
	}
cout<<endl;
size=0;
vector<uint64_t> num {219,1,245,91,123,31,29,123};

for(int i =0;i<8;i++)
        {
                size>>=8;
                cout<< size <<" , ";
		size+= (num[i]<<56);
		cout<< (num[i]<<56)<<" , "<<size<<endl;

        }
cout<<"KMagic"<<kMagic<<endl;

cout<<"size"<<size<<endl;
}



double DegToRad(double deg)
{
return M_PI * deg / 180.;
}


Eigen::Vector3d LatLongAltToEcef(const double latitude, const double longitude,
                                 const double altitude) {
  // note: 地固坐标系(Earth-Fixed Coordinate System)也称地球坐标系,
  // 是固定在地球上与地球一起旋转的坐标系.
  // 如果忽略地球潮汐和板块运动, 地面上点的坐标值在地固坐标系中是固定不变的.

  // https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#From_geodetic_to_ECEF_coordinates

  constexpr double a = 6378137.;  // semi-major axis, equator to center.
  constexpr double f = 1. / 298.257223563;
  constexpr double b = a * (1. - f);  // semi-minor axis, pole to center.
  constexpr double a_squared = a * a;
  constexpr double b_squared = b * b;
  constexpr double e_squared = (a_squared - b_squared) / a_squared;
  const double sin_phi = std::sin(DegToRad(latitude));
  const double cos_phi = std::cos(DegToRad(latitude));
  const double sin_lambda = std::sin(DegToRad(longitude));
  const double cos_lambda = std::cos(DegToRad(longitude));
  const double N = a / std::sqrt(1 - e_squared * sin_phi * sin_phi);
  const double x = (N + altitude) * cos_phi * cos_lambda;
  const double y = (N + altitude) * cos_phi * sin_lambda;
  const double z = (b_squared / a_squared * N + altitude) * sin_phi;

  return Eigen::Vector3d(x, y, z);
}

void ComputeLocalFrameFromLatLong(
    const double latitude, const double longitude) {
  const Eigen::Vector3d translation = LatLongAltToEcef(latitude, longitude, 0.);
  cout<<"\ntranslation x y z = "<<translation.transpose() <<endl;

  const Eigen::Quaterniond rotation =
      Eigen::AngleAxisd(DegToRad(latitude - 90.),
                        Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(DegToRad(-longitude),
                        Eigen::Vector3d::UnitZ());
  cout<<"rotation"
	  << rotation.w() <<" "
	  << rotation.x() <<" "
	  << rotation.y() <<" "
	  << rotation.z() <<endl;
  Eigen::Vector3d euler_angles_delta;
  euler_angles_delta = rotation.matrix().eulerAngles(0,1,2);
  cout<< "rotation matrix to roll pitch yaw (deg)" <<euler_angles_delta.transpose() * 180 / M_PI <<endl;

  Eigen::Vector3d local = rotation * -translation;
  cout<<"\nlocal xyz ="<<local.transpose()<<endl;

  Eigen::Vector3d now = rotation * translation + local;
  cout<<"\nnow xyz = "<<now.transpose()<<endl;
}



void test_fix_data()
{
ComputeLocalFrameFromLatLong(39,116);
}


int main(int argc, char **argv)
{
	google::InitGoogleLogging(argv[0]);
	google::SetStderrLogging(google::GLOG_INFO);

	LOG(INFO) <<"hello cartographer!";
	cout <<endl;

	LOG(WARNING) << "test_uint()" <<endl;
	test_uint();
	cout<<endl;
	
	LOG(WARNING) <<"test_fix_data()"<<endl;
	test_fix_data();
	cout<<endl;

	return 0;

}
