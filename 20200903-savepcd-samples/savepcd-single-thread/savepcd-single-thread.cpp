#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

#include <pcl/pcl_base.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vlp_grabber.h>

typedef pcl::PointXYZI PointType;

int loadPointCloudFromCSV(const std::string &csvname, pcl::PointCloud<pcl::PointXYZI> &cloud)
{
  std::ifstream ifs(csvname, std::ios::in);
  if (!ifs)
  {
    std::cout << "no such file" << std::endl;
    return 1; // File open failed
  }

  std::string buf;
  std::getline(ifs, buf); // Skip first line ("Points:0","Points:1","Points:2","intensity","laser_id","azimuth","distance_m","timestamp")
  while (ifs && std::getline(ifs, buf))
  {
    std::vector<std::string> v;
    boost::algorithm::split(v, buf, boost::is_any_of(","));
    if (v.size() < 4)
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
int main(int argc, char *argv[])
{
  //取得するフレーム数を決める
  int num_frame;
  std::vector<int> v_num;
  std::string FILEPATH;

  std::cout << "Please input number of frame:" << std::endl;
  std::cin >> num_frame;
  std::cout << "Please input FILEPATH:" << std::endl;
  std::cin >> FILEPATH;

  std::string ipaddress("192.168.1.201");
  std::string port("2368");
  std::string pcap;

  std::cout << "-ipadress : " << ipaddress << std::endl;
  std::cout << "-port : " << port << std::endl;

  // Set the number of frames to skip before writing one point cloud
  uint32_t writeEveryNFrames{1}; // Write a PCD file every N frames

  std::cout << "-writeEveryNFrames : " << writeEveryNFrames << std::endl;

  // Point Cloud
  pcl::PointCloud<PointType>::ConstPtr cloud;
  // No need for the mutex here because the point cloud is grabbed and processed in a single thread
  // boost::mutex mutex;

  size_t currentFrame{0};        // Current frame index
  std::atomic_uint64_t frame{0}; // Written frame index (atomic)
  time_t rcvTime;                // Receive time [us]

  // Retrieved Point Cloud Callback Function
  boost::function<void(const pcl::PointCloud<PointType>::ConstPtr &)> function =
      [&cloud, &writeEveryNFrames, &currentFrame, &frame, &rcvTime, &FILEPATH](const pcl::PointCloud<PointType>::ConstPtr &ptr) {
        /* Point Cloud Processing */
        // No need to lock this whole section runs in a single thread

        // Update the current frame counter
        ++currentFrame;

        // Write to file only every N frames
        if (currentFrame % writeEveryNFrames == 0)
        {
          // Get the receive time [us]
          rcvTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

          // Write to a PCD file
          pcl::io::savePCDFileASCII(FILEPATH + "/" + std::to_string(frame) + ".pcd", *ptr);

          std::cout << "["
                    << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
                    << "] -> frame="
                    << frame
                    << " - rcvTime="
                    << rcvTime
                    << "\n";

          ++frame;
        }
      };

  // VLP Grabber
  boost::shared_ptr<pcl::VLPGrabber> grabber;
  grabber = boost::shared_ptr<pcl::VLPGrabber>(new pcl::VLPGrabber(boost::asio::ip::address::from_string(ipaddress), boost::lexical_cast<unsigned short>(port)));

  // Register Callback Function
  boost::signals2::connection connection = grabber->registerCallback(function);

  std::cout << "Capture from Sensor..." << std::endl;

  // Start Grabber
  grabber->start();

  while (frame < num_frame)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // Stop Grabber
  grabber->stop();

  // Flush the standard output
  std::cout.flush();

  // Disconnect Callback Function
  if (connection.connected())
  {
    connection.disconnect();
  }

  return 0;
}
