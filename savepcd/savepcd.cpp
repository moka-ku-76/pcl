#include <fstream>
#include <list>
#include <string>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/projection_matrix.h>
//#include <pcl/outofcore/visualization/grid.h>
#include <pcl/recognition/linemod/line_rgbd.h>
//#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include<limits>

#include <vector>
#include <array>
#include <thread>
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <chrono>
#include <string.h>
#include <algorithm>

#include <pcl/features/moment_of_inertia_estimation.h>

typedef pcl::PointXYZI PointType;

int loadPointCloudFromCSV(const std::string &csvname, pcl::PointCloud<pcl::PointXYZI> &cloud) {
  std::ifstream ifs(csvname, std::ios::in);
  if(!ifs){
    std::cout<<"no such file"<<std::endl;
    return 1;	// File open failed
  }

  std::string buf;
  std::getline(ifs, buf);	// Skip first line ("Points:0","Points:1","Points:2","intensity","laser_id","azimuth","distance_m","timestamp")
  while(ifs && std::getline(ifs, buf)) {
    std::vector<std::string> v;
    boost::algorithm::split(v, buf, boost::is_any_of(","));
    if(v.size() < 4)
    continue;
    pcl::PointXYZI p;
    p.x = std::atof(v[0].c_str());
    p.y = std::atof(v[1].c_str());
    p.z = std::atof(v[2].c_str());
    p.intensity = std::atof(v[3].c_str());
    cloud.push_back(p);
  }
  return 0;
}

//入力するCSVファイル、点の数の配列を保存するCSVファイル名
int main( int argc, char *argv[] ){
  //取得するフレーム数を決める
  int num_frame;
  std::vector<int> v_num;
  std::string FILEPATH;

  std::cout<<"Please input number of frame:"<<std::endl;
  std::cin>>num_frame;
  std::cout<<"Please input FILEPATH:"<<std::endl;
  std::cin>>FILEPATH;


  std::string ipaddress( "192.168.1.77" );
  std::string port( "2368" );
  std::string pcap;

  std::cout << "-ipadress : " << ipaddress << std::endl;
  std::cout << "-port : " << port << std::endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
  // Point Cloud
  pcl::PointCloud<PointType>::ConstPtr cloud;

  // Retrieved Point Cloud Callback Function
  boost::mutex mutex;
  boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function =
  [ &cloud, &mutex ]( const pcl::PointCloud<PointType>::ConstPtr& ptr ){
    boost::mutex::scoped_lock lock( mutex );
    /* Point Cloud Processing */
    cloud = ptr;
  };
  // VLP Grabber
  boost::shared_ptr<pcl::VLPGrabber> grabber;

  std::cout << "Capture from Sensor..." << std::endl;
  grabber = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );

  // Register Callback Function
  boost::signals2::connection connection = grabber->registerCallback( function );

  // Start Grabber
  grabber->start();
  // boost::timer t;
  int frame=0;
  // int save_frame=num_frame/5;
  while( frame<num_frame ){
    viewer->spinOnce();
    boost::mutex::scoped_try_lock lock( mutex );
    if( lock.owns_lock() && cloud ){
      pcl::io::savePCDFileASCII(FILEPATH+"/"+std::to_string(frame)+".pcd",*cloud);

      frame++;
      std::cout<<"frame="<<frame<<std::endl;
    }
  }

  // Stop Grabber
  grabber->stop();

  // Disconnect Callback Function
  if( connection.connected() ){
    connection.disconnect();
  }

  return 0;
}
