#include <ros/ros.h>
#include <cmath>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <filesystem>

using namespace std;
using namespace cv;

bool worldRunning = false;
int network_width = 256;
int network_height = 256;
int fileCount = 0;

void setFileCount(){
   std::filesystem::path imagePath { "/home/ryan/TrainingData/semantic_segmentation/img" };
   for (auto& p : std::filesystem::directory_iterator(imagePath))
   {
      ++fileCount;
   }
 }
class SemanticSegmentationRecorder{
    private:
      ros::NodeHandle n;
      ros::Subscriber controls_sub;
      image_transport::ImageTransport image_transport;
      image_transport::Subscriber image_sub;
      int movement_direction = 0; // 0 = none 1 = left 2 = right 3 = up 4 = down
      int rotation_direction = 0; // -1 = left 0 = none 1 = right
      int move_speed = 1;

    public:
      SemanticSegmentationRecorder():image_transport(n){
          n = ros::NodeHandle("~");
          std::string camera_topic;
          n.getParam("/camera_topic",camera_topic);
          n.getParam("/network_width",network_width);
          n.getParam("/network_height",network_height);
          image_sub = image_transport.subscribe(camera_topic, 1, &SemanticSegmentationRecorder::image_callback, this);
          controls_sub = n.subscribe("/simulator/command/input",100,&SemanticSegmentationRecorder::command_callback,this);
      }
      ~SemanticSegmentationRecorder()
      {
      }
      Mat getMask(Mat input, int r, int g, int b){
        Scalar lower(r,g,b);
        Scalar upper(r,g,b);
        Mat colorFiltered;
        inRange(input,lower,upper,colorFiltered);
        return colorFiltered;
      }
      void command_callback(const std_msgs::Int64ConstPtr &msg){
        if(((int)msg->data) == 116){
          // t
            if(worldRunning){
              ROS_INFO("World Paused: IOSTREAM CLOSED");
              worldRunning = false;
            }
            else{
              ROS_INFO("World Resumed: IOSTREAM OPENED");
              worldRunning = true;
            }
        }
        if(((int)msg->data) == 113){
          //q
          //rotate left
          this->rotation_direction = -1;
        }
        if(((int)msg->data) == 119){
          //w
          //move forward
          this->movement_direction = 3;
        }
        if(((int)msg->data) == 101){
          //e
          // rotate right
          this->rotation_direction = 1;
        }
        if(((int)msg->data) == 97){
          //a
          // move left
          this->movement_direction = 1;
        }
        if(((int)msg->data) == 115){
          //s
          // move down
          this->movement_direction = 4;
        }
        if(((int)msg->data) == 100){
          //d
          // move right
          this->movement_direction = 2;
        }
        if(((int)msg->data) == 114){
          //r
          // stop rotation
          this->rotation_direction = 0;
        }
        if(((int)msg->data) == 32){
          // space 
          // stop moving
          this->movement_direction = 0;
        }
        if(((int)msg->data) == 122){
          //z
          //speed up
          this->move_speed = this->move_speed + 1;
        }
        if(((int)msg->data) == 120){
          //x
          //slow down
          this->move_speed = this->move_speed - 1;
          if(this->move_speed<=0){
            this->move_speed = 1;
          }
        }
    }
      void image_callback(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat outputMat;
        try
        {
          if(worldRunning){
            cv_ptr = cv_bridge::toCvCopy(msg, "");
            Mat imageInRGB = cv_ptr->image;
            Mat imageInBGR;
            cv::cvtColor(imageInRGB,imageInBGR,cv::COLOR_RGB2BGR);

            //must convert to bgr before saving
            cv::imwrite(("/home/ryan/TrainingData/semantic_segmentation/img/"+to_string(fileCount+1) + ".png"),imageInBGR);

            Mat yellowLaneLines = getMask(imageInRGB,255,229, 51);

            Mat whiteLaneLines = getMask(imageInRGB,255,255,255);

            //car colors
            Mat blueCarMat = getMask(imageInRGB,3,2,255);
            Mat redCarMat = getMask(imageInRGB,255,0,1);
            Mat yellowCarMat = getMask(imageInRGB,206,194,0);
            Mat clearCarMat = getMask(imageInRGB,61,61,61);
            Mat blackCarMat = getMask(imageInRGB,0,0,0);
            
            Mat carMiddleSection = getMask(imageInRGB,5,5,5);
            Mat lidarAndAntenna = getMask(imageInRGB,2,2,2);
            Mat powerBoard = getMask(imageInRGB,5,0,15);
            Mat whitePartOfCar = getMask(imageInRGB,168,168,168);
            Mat wheels = getMask(imageInRGB,76,76,76);

            Mat cars = blueCarMat |redCarMat | yellowCarMat | clearCarMat | blackCarMat | carMiddleSection | lidarAndAntenna | powerBoard | whitePartOfCar | wheels;
            
            //scaling
            cars = cars * 2 /256;
            yellowLaneLines = yellowLaneLines * 3/256;
            whiteLaneLines = whiteLaneLines * 1/256;

            Mat combinedMat = cars | yellowLaneLines | whiteLaneLines;
            cv::imwrite(("/home/ryan/TrainingData/semantic_segmentation/masks_machine/"+to_string(fileCount+1) + ".png"),combinedMat);
            fileCount++;
          }
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
      }
};

int main(int argc, char ** argv) {
    setFileCount();
    ros::init(argc, argv, "semantic_segmentation_file_recorder");
    SemanticSegmentationRecorder rw;
    ros::spin();
    return 0;
}
