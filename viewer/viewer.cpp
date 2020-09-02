#include <pcl/visualization/cloud_viewer.h>
#include <pcl/recognition/linemod/line_rgbd.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZ PointType;

int user_data;

// void viewerOneOff (pcl::visualization::PCLVisualizer& viewer){
//     viewer.setBackgroundColor (0, 0, 0);
//     pcl::PointXYZ o;
//     o.x = 1.0;
//     o.y = 0;
//     o.z = 0;
//     viewer.addSphere (o, 0.25, "sphere", 0);
//     std::cout << "i only run once" << std::endl;
//
// }
//
// void viewerPsycho (pcl::visualization::PCLVisualizer& viewer){
//     static unsigned count = 0;
//     std::stringstream ss;
//     ss << "Once per viewer loop: " << count++;
//     viewer.removeShape ("text", 0);
//     viewer.addText (ss.str(), 200, 300, "text", 0);
//
//     //FIXME: possible race condition here:
//     user_data++;
// }
pcl::PointCloud<PointType>::Ptr makegrid(float min[3],float max[3]){
  pcl::PointCloud<PointType>::Ptr cloud_grid(new pcl::PointCloud<PointType>);

  cloud_grid->width=8;
  cloud_grid->height=1;
  cloud_grid->is_dense=true;
  cloud_grid->points.resize(cloud_grid->width*cloud_grid->height);

  cloud_grid->points;
  for(int i=0;i<8;i++){
    if(i%2==0){
      cloud_grid->points[i].x=min[0];
    }else {
      cloud_grid->points[i].x=max[0];
    }
    if(i%4<2){
      cloud_grid->points[i].y=min[1];
    }else {
      cloud_grid->points[i].y=max[1];
    }
    if(i<4){
      cloud_grid->points[i].z=min[2];
    }else{
      cloud_grid->points[i].z=max[2];
    }
  }

  return cloud_grid;
}

// pcl::visualization::PCLVisualizer highlight(pcl::visualization::PCLVisualizer viewer,pcl::PointCloud<PointType>::Ptr cloud_grid,int vp[6]){
//
// for(int i=0;i<6;i++){
//   viewer.addLine<PointType>(cloud_grid->points[0],cloud_grid->points[1],255,255,0,"sen1"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[1],cloud_grid->points[3],255,255,0,"sen2"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[3],cloud_grid->points[2],255,255,0,"sen3"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[2],cloud_grid->points[0],255,255,0,"sen4"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[0],cloud_grid->points[4],255,255,0,"sen5"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[1],cloud_grid->points[5],255,255,0,"sen6"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[3],cloud_grid->points[7],255,255,0,"sen7"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[2],cloud_grid->points[6],255,255,0,"sen8"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[4],cloud_grid->points[5],255,255,0,"sen9"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[5],cloud_grid->points[7],255,255,0,"sen10"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[7],cloud_grid->points[6],255,255,0,"sen11"+std::to_string(i),vp[i]);
//   viewer.addLine<PointType>(cloud_grid->points[6],cloud_grid->points[4],255,255,0,"sen12"+std::to_string(i),vp[i]);
// }
//
//   return viewer;
// }

void highlight(pcl::visualization::PCLVisualizer viewer,pcl::PointCloud<PointType>::Ptr cloud_grid,int vp[6],int frame){

  for(int i=0;i<6;i++){
    viewer.addLine<PointType>(cloud_grid->points[0],cloud_grid->points[1],230, 230, 20,"sen1"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[1],cloud_grid->points[3],230, 230, 20,"sen2"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[3],cloud_grid->points[2],230, 230, 20,"sen3"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[2],cloud_grid->points[0],230, 230, 20,"sen4"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[0],cloud_grid->points[4],230, 230, 20,"sen5"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[1],cloud_grid->points[5],230, 230, 20,"sen6"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[3],cloud_grid->points[7],230, 230, 20,"sen7"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[2],cloud_grid->points[6],230, 230, 20,"sen8"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[4],cloud_grid->points[5],230, 230, 20,"sen9"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[5],cloud_grid->points[7],230, 230, 20,"sen10"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[7],cloud_grid->points[6],230, 230, 20,"sen11"+std::to_string(i)+std::to_string(frame),vp[i]);
    viewer.addLine<PointType>(cloud_grid->points[6],cloud_grid->points[4],230, 230, 20,"sen12"+std::to_string(i)+std::to_string(frame),vp[i]);
  }
}


//データが保存されているフォルダを選択することでそのフォルダ内のpcdファイルを２つずつ見ることができる
int main (int argc,char** argv){
    pcl::PointCloud<PointType>::Ptr cloud_grid(new pcl::PointCloud<PointType>);
    std::string FILEPATH;
    FILEPATH=argv[1];
    int frame=0;
    pcl::PointCloud<PointType>::Ptr cloud1 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud2 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud3 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud4 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud5 (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud6 (new pcl::PointCloud<PointType>);

    float min[3]={-1.2,0,-0.01};
    float max[3]={0.3,1.5,0.2};

    cloud_grid=makegrid(min,max);
    int interval;
    std::cout<<"Please input number of interval:"<<std::endl;
    std::cin>>interval;


    while(frame<10000){

      // pcl::PointCloud<PointType>::Ptr cloud7 (new pcl::PointCloud<PointType>);
      // pcl::PointCloud<PointType>::Ptr cloud8 (new pcl::PointCloud<PointType>);
      // pcl::PointCloud<PointType>::Ptr cloud9 (new pcl::PointCloud<PointType>);
      // pcl::PointCloud<PointType>::Ptr cloud10 (new pcl::PointCloud<PointType>);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame)+".pcd", *cloud1);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+interval)+".pcd", *cloud2);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+(2*interval))+".pcd", *cloud3);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+(3*interval))+".pcd", *cloud4);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+(4*interval))+".pcd", *cloud5);
      pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+(5*interval))+".pcd", *cloud6);
      // pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+6)+".pcd", *cloud7);
      // pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+7)+".pcd", *cloud8);
      // pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+8)+".pcd", *cloud9);
      // pcl::io::loadPCDFile (FILEPATH+"/"+std::to_string(frame+9)+".pcd", *cloud10);

      int vp[6];
      pcl::visualization::PCLVisualizer viewer("Viewer");
      pcl::visualization::PointCloudColorHandlerCustom<PointType> grid_cloud_color_handler (cloud1, 230, 230, 20);

      //  pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb1(cloud1);
      //  pcl::visualization::PointCloudColorHandlerRGBField<PointTypeRGB> rgb2(cloud2);
      viewer.createViewPort(0.0, 0.5, 0.33, 1.0, vp[0]);
      viewer.createViewPort(0.33, 0.5, 0.66, 1.0, vp[1]);
      viewer.createViewPort(0.66, 0.5, 1.0, 1.0, vp[2]);
      viewer.createViewPort(0.0, 0.0, 0.33, 0.5, vp[3]);
      viewer.createViewPort(0.33, 0.0, 0.66, 0.5, vp[4]);
      viewer.createViewPort(0.66, 0.0, 1.0, 0.5, vp[5]);
      // viewer.createViewPort(0.2, 0.0, 0.4, 0.5, vp[6]);
      // viewer.createViewPort(0.4, 0.0, 0.6, 0.5, vp[7]);
      // viewer.createViewPort(0.6, 0.0, 0.8, 0.5, vp[8]);
      // viewer.createViewPort(0.8, 0.0, 1.0, 0.5, vp[9]);
      viewer.addPointCloud(cloud1, "frame+0", vp[0]);
      viewer.addPointCloud(cloud2, "frame+1", vp[1]);
      viewer.addPointCloud(cloud3, "frame+2", vp[2]);
      viewer.addPointCloud(cloud4, "frame+3", vp[3]);
      viewer.addPointCloud(cloud5, "frame+4", vp[4]);
      viewer.addPointCloud(cloud6, "frame+5", vp[5]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid0", vp[0]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid1", vp[1]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid2", vp[2]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid3", vp[3]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid4", vp[4]);
      viewer.addPointCloud(cloud_grid, grid_cloud_color_handler,"grid5", vp[5]);


      pcl::PointXYZ o1;
      o1.x=0;
      o1.y=0;
      o1.z=0;
      pcl::PointXYZ o2;
      o2.x=0.55;
      o2.y=0.77;
      o2.z=0;
      for(int i=0;i<6;i++){
        viewer.addSphere(o1,0.05,"sensor1"+std::to_string(i)+std::to_string(frame),vp[i]);
        viewer.addSphere(o2,0.05,"sensor2"+std::to_string(i)+std::to_string(frame),vp[i]);

      }
      highlight(viewer,cloud_grid,vp,frame);

      // viewer.addPointCloud(cloud7, "frame+6", vp[6]);
      // viewer.addPointCloud(cloud8, "frame+7", vp[7]);
      // viewer.addPointCloud(cloud9, "frame+8", vp[8]);
      // viewer.addPointCloud(cloud9, "frame+9", vp[9]);
      viewer.addText("0", 10, 10, "frame+0", vp[0]);
      viewer.addText("1", 10, 10, "frame+1", vp[1]);
      viewer.addText("2", 10, 10, "frame+2", vp[2]);
      viewer.addText("3", 10, 10, "frame+3", vp[3]);
      viewer.addText("4", 10, 10, "frame+4", vp[4]);
      viewer.addText("5", 10, 10, "frame+5", vp[5]);
      // viewer.addText("6", 10, 10, "frame+6", vp[6]);
      // viewer.addText("7", 10, 10, "frame+7", vp[7]);
      // viewer.addText("8", 10, 10, "frame+8", vp[8]);
      // viewer.addText("9", 10, 10, "frame+9", vp[9]);

      viewer.spin();

      std::cerr<<"Please input frame number"<<std::endl;
      std::cin>>frame;
     }

    return 0;
}
