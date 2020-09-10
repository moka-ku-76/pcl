#include <chrono>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
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

  // Point Cloud queue
  std::queue<pcl::PointCloud<PointType>> pointClouds;
  // Receive time queue
  std::queue<time_t> rcvTimes;
  boost::mutex mutex;

  size_t currentFrame{0};              // Current frame index (atomic)
  std::atomic_uint64_t toBeWritten{0}; // Indicates how many point clouds remains to be written to a PCD file (atomic)

  // Retrieved Point Cloud Callback Function
  boost::function<void(const pcl::PointCloud<PointType>::ConstPtr &)> function =
      [&pointClouds, &writeEveryNFrames, &mutex, &currentFrame, &rcvTimes, &toBeWritten](const pcl::PointCloud<PointType>::ConstPtr &ptr) {
        /* Point Cloud Processing */

        // Update the current frame index
        ++currentFrame;

        // Store one frame every N frames
        if (currentFrame % writeEveryNFrames == 0)
        {
          // Lock the access to the queues
          boost::mutex::scoped_lock lock(mutex);

          // Store the corresponding receive time [us]
          rcvTimes.push(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count());

          // Enqueue a copy of the point cloud
          pointClouds.push(*ptr);

          // Update the number of point clouds that has to be written
          toBeWritten = pointClouds.size();
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

  time_t rcvTime;  // Receiving time [us]
  size_t frame{0}; // Written frame index
  pcl::PointCloud<PointType> cloud;

  while (frame < num_frame)
  {
    if (toBeWritten)
    {
      // Acquire the lock immediately or wait for the point cloud update to be finished
      boost::mutex::scoped_lock lock(mutex);

      // Get the first point cloud in the queue while being protected by the lock
      cloud = std::move(pointClouds.front());
      pointClouds.pop();

      // Get the corresponding receive time [us]
      rcvTime = rcvTimes.front();
      rcvTimes.pop();

      // Update the number of point clouds that has to be written
      --toBeWritten;
    }
    // Release the lock automatically here because it goes out of scope

    if (cloud.empty())
    {
      // Sleep for 1 ms before trying to grab a new point cloud
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    pcl::io::savePCDFileASCII(FILEPATH + "/" + std::to_string(frame) + ".pcd", cloud);

    // Uncomment this line to mimic a longer writing time
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "["
              << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
              << "] -> frame="
              << frame
              << " - rcvTime="
              << rcvTime
              << " (waiting in queue="
              << toBeWritten
              << ")\n";

    cloud.clear();

    frame++;
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
