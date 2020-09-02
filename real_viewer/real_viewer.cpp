#include <fstream>
#include <list>
#include <string>
#include <boost/algorithm/string.hpp>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <iostream>
#include <stdio.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include<limits>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZI PointType;

pcl::PointCloud<PointType>::Ptr merge_cloud(pcl::PointCloud<PointType>::Ptr cloud1, pcl::PointCloud<PointType>::Ptr cloud2){
  pcl::PointCloud<PointType>::Ptr cloud_merged(new pcl::PointCloud<PointType>);//２つの点群をまとめて格納する
  *cloud_merged = *cloud1;//１つ目の点群を統合先に格納する
  *cloud_merged += *cloud2;//２つ目の点群を統合先に格納する。

  return cloud_merged;
}
int loadPointCloudFromCSV(const std::string &csvname, pcl::PointCloud<PointType> &cloud) {
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

pcl::PointCloud<PointType>::Ptr cut(pcl::PointCloud<PointType>::Ptr cloud){
  // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<PointType>::Ptr cutCloud(new pcl::PointCloud<PointType>);

  float inf=std::numeric_limits<float>::infinity();
  float x_min=-1.2;
  float x_max=0.3;
  float y_min=0;
  float y_max=1.5;
  float z_min=-0.018;
  float z_max=0.2;

  //build the condition
  pcl::ConditionAnd<PointType>::Ptr range_cond(new pcl::ConditionAnd<PointType>());

  //build the conditions
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x",pcl::ComparisonOps::GT,x_min)));
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("x",pcl::ComparisonOps::LT,x_max)));
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y",pcl::ComparisonOps::GT,y_min)));
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("y",pcl::ComparisonOps::LT,y_max)));
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z",pcl::ComparisonOps::GT,z_min)));
  range_cond->addComparison(pcl::FieldComparison<PointType>::ConstPtr(new pcl::FieldComparison<PointType>("z",pcl::ComparisonOps::LT,z_max)));
  //build the filter
  pcl::ConditionalRemoval<PointType>condrem;
  condrem.setCondition(range_cond);
  condrem.setInputCloud(cloud);
  condrem.setKeepOrganized(true);
  //apply filter
  condrem.filter(*cutCloud);

  std::vector<int> mapping;
  pcl::removeNaNFromPointCloud(*cutCloud, *cutCloud, mapping);

  return cutCloud;
}

//点群を移動させる関数
//参考：http://pointclouds.org/documentation/tutorials/matrix_transform.php
pcl::PointCloud<PointType>::Ptr transform(pcl::PointCloud<PointType>::Ptr cloud,float x_bias,float y_bias,float z_bias,float theta){
  pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();

  // Define a translation of bias on the each axis.
  transform.translation() << x_bias, y_bias, z_bias;

  // The same rotation matrix as before; theta radians around Z axis
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  //printf ("\nMethod #2: using an Affine3f\n");
  //std::cout << transform.matrix() << std::endl;

  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud, *transformed_cloud, transform);

  return transformed_cloud;
}


//入力するCSVファイル、点の数の配列を保存するCSVファイル名
int main( int argc, char *argv[] ){
  //std::string count_file_name=argv[1];
  // std::string change_file_name=argv[2];
  //取得するフレーム数を決める
  int num_frame;
  std::vector<int> v_num;
  std::string FILEPATH;
  float x_bias=0.5;
  float y_bias=0.79;
  float z_bias=0;
  float theta=0;

  std::cout<<"Please input number of frame:"<<std::endl;
  std::cin>>num_frame;
  std::cout<<"Please input FILEPATH:"<<std::endl;
  std::cin>>FILEPATH;


  std::string ipaddress1( "192.168.1.77" );
  std::string ipaddress2( "192.168.2.77" );
  std::string port1( "2368" );
  std::string port2( "2268" );

  std::cout << "-ipadress1 : " << ipaddress1 << std::endl;
  std::cout << "-port1 : " << port1 << std::endl;
  std::cout << "-ipadress2 : " << ipaddress2 << std::endl;
  std::cout << "-port2 : " << port2 << std::endl;


  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );

  // Point Cloud
  pcl::PointCloud<PointType>::ConstPtr cloud1;
  pcl::PointCloud<PointType>::ConstPtr cloud2;

  // Retrieved Point Cloud Callback Function
  boost::mutex mutex1;
  boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function1 =
  [ &cloud1, &mutex1 ]( const pcl::PointCloud<PointType>::ConstPtr& ptr1 ){
    boost::mutex::scoped_lock lock( mutex1 );

    /* Point Cloud Processing */

    cloud1 = ptr1;
  };

  boost::mutex mutex2;
  boost::function<void( const pcl::PointCloud<PointType>::ConstPtr& )> function2 =
  [ &cloud2, &mutex2 ]( const pcl::PointCloud<PointType>::ConstPtr& ptr2 ){
    boost::mutex::scoped_lock lock( mutex2 );

    /* Point Cloud Processing */

    cloud2 = ptr2;
  };

  pcl::visualization::PointCloudColorHandler<PointType>::Ptr handler;
  std::vector<double> color = { 255.0, 255.0, 255.0 };
  boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<PointType>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<PointType>( color[0], color[1], color[2] ) );
  handler = color_handler;

  // VLP Grabber
  boost::shared_ptr<pcl::VLPGrabber> grabber1;
  boost::shared_ptr<pcl::VLPGrabber> grabber2;

  std::cout << "Capture from Sensor..." << std::endl;
  grabber1 = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress1 ), boost::lexical_cast<unsigned short>( port1 ) ) );
  grabber2 = boost::shared_ptr<pcl::VLPGrabber>( new pcl::VLPGrabber( boost::asio::ip::address::from_string( ipaddress2 ), boost::lexical_cast<unsigned short>( port2 ) ) );

  // Register Callback Function
  boost::signals2::connection connection1 = grabber1->registerCallback( function1 );
  boost::signals2::connection connection2 = grabber2->registerCallback( function2 );

  // Start Grabber
  grabber1->start();
  grabber2->start();
  // boost::timer t;
  int frame=0;
  while( frame<num_frame ){
    viewer->spinOnce();
    boost::mutex::scoped_try_lock lock1( mutex1 );
    boost::mutex::scoped_try_lock lock2( mutex2 );
    if( lock1.owns_lock() && cloud1 ){
      // Update Point Cloud
      // handler->setInputCloud( cloud );
      pcl::PointCloud<PointType>::Ptr Cloud1(new pcl::PointCloud<PointType>);
      *Cloud1=*cloud1;
      pcl::PointCloud<PointType>::Ptr cutCloud1(new pcl::PointCloud<PointType>);
      cutCloud1=cut(Cloud1);

      if( lock2.owns_lock() && cloud2 ){
        // Update Point Cloud
        // handler->setInputCloud( cloud );

        pcl::PointCloud<PointType>::Ptr Cloud2(new pcl::PointCloud<PointType>);
        *Cloud2=*cloud2;

        pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType>);
        transformed_cloud=transform(Cloud2,x_bias,y_bias,z_bias,theta);

        pcl::PointCloud<PointType>::Ptr cutCloud2(new pcl::PointCloud<PointType>);
        cutCloud2=cut(transformed_cloud);

        pcl::PointCloud<PointType>::Ptr merged_cloud(new pcl::PointCloud<PointType>);
        merged_cloud=merge_cloud(cutCloud1,cutCloud2);
        //pcl::io::savePCDFileASCII(FILEPATH+"/sensor1/"+std::to_string(frame)+".pcd",*cutCloud1);
        //pcl::io::savePCDFileASCII(FILEPATH+"/sensor2/"+std::to_string(frame)+".pcd",*cutCloud2);
        //pcl::io::savePCDFileASCII(FILEPATH+"/integrate/"+std::to_string(frame)+".pcd",*merged_cloud);
        //pcl::io::savePCDFileASCII(FILEPATH+"/"+std::to_string(frame)+".pcd",*merged_cloud);

        handler->setInputCloud( merged_cloud );
        if( !viewer->updatePointCloud( merged_cloud, *handler, "cloud" ) ){
            viewer->addPointCloud( merged_cloud, *handler, "cloud" );
        }


        frame++;
        std::cout<<"frame="<<frame<<std::endl;
      }

    }

  }

  // Stop Grabber
  grabber1->stop();
  grabber2->stop();

  // Disconnect Callback Function
  if( connection1.connected() ){
    connection1.disconnect();
  }
  if( connection2.connected() ){
    connection2.disconnect();
  }

  return 0;
}
