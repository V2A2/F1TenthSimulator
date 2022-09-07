#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/ModelState.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Float64MultiArray.h>
#include <Eigen/Dense>
#include <cmath>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/Image.h>
#include <fstream>
#include <std_msgs/Int64.h>
#include <filesystem>
#include <std_msgs/Float64MultiArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;
using namespace Eigen;
ofstream boundingBoxFile;
int boundingBoxFileCount = 0;
int possible_bounding_boxes_for_image = 10;
int desired_x = 1280;
int desired_y = 720;
bool include_pose = true;
bool worldRunning = false;
void setFileCount(){
   std::filesystem::path followingFilePath { "/home/ryan/TrainingData/object_detection/photos" };
   for (auto& p : std::filesystem::directory_iterator(followingFilePath))
   {
      ++boundingBoxFileCount;
   }
 }
class ObjectData{
  public:
    std::vector<Vector3d> corners_for_model(string model){
      std::vector<Vector3d> corners;
      if(model == "red_car" || model == "blue_car" || model == "yellow_car" || model == "black_car" || model == "clear_car"){
        //base
        Vector3d corner1(0.17,0.1,0.05);
        Vector3d corner2(0.17,-0.1,0.05);
        Vector3d corner3(-0.37,0.1,0.05);
        Vector3d corner4(-0.37,-0.1,0.05);
        Vector3d corner5(0.17,0.1,0.15);
        Vector3d corner6(0.17,-0.1,0.15);
        Vector3d corner7(-0.37,0.1,0.15);
        Vector3d corner8(-0.37,-0.1,0.15);
        //camera
        Vector3d corner9(-.03,-.05,.23);
        Vector3d corner10(-.03,.05,.23);
        //antenna
        Vector3d corner11(-.23,-.08,.29);
        Vector3d corner12(-.23,.08,.29);
        //Wheels
        Vector3d corner13(-0.28,-0.12,0.05);
        Vector3d corner14(-0.28,-0.12,0.0);
        Vector3d corner15(-0.28,0.12,0.05);
        Vector3d corner16(-0.28,0.12,0.0);
        Vector3d corner17(0.09,-0.12,0.05);
        Vector3d corner18(0.09,-0.12,0.0);
        Vector3d corner19(0.09,0.12,0.05);
        Vector3d corner20(0.09,0.12,0.0);

        corners.push_back(corner1);
        corners.push_back(corner2);
        corners.push_back(corner3);
        corners.push_back(corner4);
        corners.push_back(corner5);
        corners.push_back(corner6);
        corners.push_back(corner7);
        corners.push_back(corner8);
        corners.push_back(corner9);
        corners.push_back(corner10);
        corners.push_back(corner11);
        corners.push_back(corner12);
        corners.push_back(corner13);
        corners.push_back(corner14);
        corners.push_back(corner15);
        corners.push_back(corner16);
        corners.push_back(corner17);
        corners.push_back(corner18);
        corners.push_back(corner19);
        corners.push_back(corner20);
      }
      return corners;
    }
    int classification(string model){
      if(model == "red_car" || model == "blue_car" || model == "yellow_car" || model == "black_car" || model == "clear_car"){
        return 0;
      }
      return -1;
    }
};
class ObjectTracking{
    private:
      ros::NodeHandle n;
      ros::Subscriber model_subscriber;
      ros::Subscriber controls_sub;
      image_transport::ImageTransport image_transport;
      image_transport::Subscriber image_sub;
      image_transport::Publisher image_pub;
      ros::Publisher car_movement_pub;
      ObjectData object_data_finder;
      std::string car_name;
      std::string moving_car_name;
      geometry_msgs::Quaternion moving_car_orientation;
      geometry_msgs::Point moving_car_position;
      double camera_x = 0;
      double camera_y = 0;
      double camera_z = 0;
      double camera_roll = 0;
      double camera_pitch = 0;
      double camera_yaw = 0;
      double cameraHorizontalFocalLength = 674.4192802;//649.4571919; Simulator only uses 1 focal length but real camera uses 649.457 for horizontal
      double cameraVerticalFocalLength = 674.4192802;
      double camera_x_in_car_frame = -0.02;
      double camera_y_in_car_frame = -0.003;
      double camera_z_in_car_frame = 0.2104;
      int camera_pixel_height = 720;
      int camera_pixel_width = 1280;
      double maxCameraDistance = 100;
      std::vector<double> lastBoundingBoxData;
      std::vector<double> lastOrientationData;
      //test vector
      std::vector<Vector2d> cv_data_points;
      bool addPoseToBoundingBoxMessage = true;
      int movement_direction = 0; // 0 = none 1 = left 2 = right 3 = up 4 = down
      int rotation_direction = 0; // -1 = left 0 = none 1 = right
      int move_speed = 1;
    public:
      ObjectTracking():image_transport(n){
          n = ros::NodeHandle("~");
          std::string model_topic,data_topic, bounding_box_topic;
          n.getParam("/model_topic", model_topic);
          n.getParam("/car_name", car_name);
          n.getParam("/moving_car_name",moving_car_name);
          n.getParam("/bounding_box_topic",bounding_box_topic);
          model_subscriber = n.subscribe(model_topic, 1, &ObjectTracking::model_state_callback,this);
          std::string camera_topic, publish_topic;
          n.getParam("/camera_topic",camera_topic);
          image_sub = image_transport.subscribe(camera_topic, 1, &ObjectTracking::image_callback, this);
          n.getParam("/image_publish_topic", publish_topic);
          image_pub = image_transport.advertise(publish_topic, 1);
          controls_sub = n.subscribe("/simulator/command/input",100,&ObjectTracking::command_callback,this);
          car_movement_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
      }
      ~ObjectTracking()
      {
      }
      void command_callback(const std_msgs::Int64ConstPtr &msg){
        if(((int)msg->data) == 116){
          // t
            if(worldRunning){
              ROS_INFO("World Paused: IOSTREAM CLOSED");
              worldRunning = false;
              boundingBoxFile.close();
              boundingBoxFile.clear();
            }
            else{
              ROS_INFO("World Resumed: IOSTREAM OPENED");
              if(!boundingBoxFile.is_open()){
                boundingBoxFile.open("/home/ryan/TrainingData/object_detection/object_detection_data.bin", ios::out | ios::app | ios::binary);
              }
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
          cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
          cv::Mat greyScale;
          cv_ptr->image.convertTo(greyScale,CV_8UC1,255,0);
          cvtColor(greyScale,outputMat,cv::COLOR_GRAY2RGB);
          //REMOVE LATER
          cv::resize(outputMat,outputMat,cv::Size(desired_x,desired_y));
          //END REMOVE
          int boundingBoxIndex = 5 + (int)(addPoseToBoundingBoxMessage) * 6;
          for(int i=0;i<static_cast<int>(lastBoundingBoxData.size());i+=boundingBoxIndex){
            cv::Point pt1(lastBoundingBoxData.at(i+1)*desired_x/this->camera_pixel_width,(this->camera_pixel_height-lastBoundingBoxData.at(i+3))*desired_y/this->camera_pixel_height );
            cv::Point pt2(lastBoundingBoxData.at(i+2)*desired_x/this->camera_pixel_width,(this->camera_pixel_height-lastBoundingBoxData.at(i+4))*desired_y/this->camera_pixel_height );
            cv::rectangle(outputMat, pt1, pt2, cv::Scalar(255, 0, 0));
            // cv::Point pt1(lastBoundingBoxData.at(i+1),this->camera_pixel_height-lastBoundingBoxData.at(i+3));
            // cv::Point pt2(lastBoundingBoxData.at(i+2),this->camera_pixel_height-lastBoundingBoxData.at(i+4));
            // cv::rectangle(outputMat, pt1, pt2, cv::Scalar(255, 0, 0));
          }
          //TEST RECORD AREA
          if(worldRunning){
            recordData(cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1)->image);
          }
          //END TEST RECORD AREA
          geometry_msgs::Quaternion new_moving_car_orientation = this->moving_car_orientation;
          geometry_msgs::Point new_moving_car_position = this->moving_car_position;
          if(this->movement_direction == 1){
            new_moving_car_position.y = new_moving_car_position.y + 0.01 * this->move_speed;
          }
          if(this->movement_direction == 2){
            new_moving_car_position.y = new_moving_car_position.y - 0.01 * this->move_speed;
          }
          if(this->movement_direction == 3){
            new_moving_car_position.x = new_moving_car_position.x + 0.01 * this->move_speed;
          }
          if(this->movement_direction == 4){
            new_moving_car_position.x = new_moving_car_position.x - 0.01 * this->move_speed;
          }
          if(this->rotation_direction == -1){
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(new_moving_car_orientation , q_orig);
            double r=0, p=0, y=0.0174533 * this->move_speed;
            q_rot.setRPY(r, p, y);
            q_new = q_rot*q_orig;
            q_new.normalize();
            tf2::convert(q_new, new_moving_car_orientation);
          }
          if(this->rotation_direction == 1){
            tf2::Quaternion q_orig, q_rot, q_new;
            tf2::convert(new_moving_car_orientation , q_orig);
            double r=0, p=0, y=-0.0174533 * this->move_speed;
            q_rot.setRPY(r, p, y);
            q_new = q_rot*q_orig;
            q_new.normalize();
            tf2::convert(q_new, new_moving_car_orientation);
          }
          geometry_msgs::Pose new_pose;
          new_pose.orientation = new_moving_car_orientation;
          new_pose.position = new_moving_car_position;
          gazebo_msgs::ModelState model_state;
          model_state.model_name = this->moving_car_name;
          model_state.pose = new_pose;
          model_state.reference_frame = "world";
          // model_state.twist
          car_movement_pub.publish(model_state);
        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }
        //Upload Mat Image as ROS TOPIC 
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->header; 
        out_msg.encoding = sensor_msgs::image_encodings::RGB8; 
        out_msg.image    = outputMat;
        //Output modified video stream
        image_pub.publish(out_msg.toImageMsg());
      }
      void find_bounding_boxes(std::vector<geometry_msgs::Pose>poses,std::vector<std::string>names){
        std::vector<std::vector<Vector3d>> cornerDataForObjects;
        std::vector<int> classifications;
        std::vector<std::vector<double>> objectPoses;
        for(int i =0; i<static_cast<int>(poses.size());i++){
          int classification = object_data_finder.classification(names.at(i));
          if(classification >= 0){
            classifications.push_back(classification);
            geometry_msgs::Quaternion orientation = poses.at(i).orientation;
            tf2::Quaternion quaternion(orientation.x,orientation.y,orientation.z,orientation.w);
            tf2::Matrix3x3 m(quaternion);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            double x = poses.at(i).position.x;
            double y = poses.at(i).position.y;
            double z = poses.at(i).position.z;
            std::vector<Vector3d> corners = getCornersInWorldFrame(names.at(i),x,y,z,roll,pitch,yaw);
            cornerDataForObjects.push_back(corners);

            std::vector<double> objectPoseInCameraFrame;
            Vector3d objectPoseInWorldFrame(x,y,z);
            Vector3d centerInCameraFrame = rotate(translate(objectPoseInWorldFrame,-this->camera_x,-this->camera_y,-this->camera_z),-this->camera_yaw,-this->camera_pitch,-this->camera_roll);
            objectPoseInCameraFrame.push_back(centerInCameraFrame.x());
            objectPoseInCameraFrame.push_back(centerInCameraFrame.y());
            objectPoseInCameraFrame.push_back(centerInCameraFrame.z());
            objectPoseInCameraFrame.push_back(roll-this->camera_roll);
            objectPoseInCameraFrame.push_back(pitch-this->camera_pitch);
            objectPoseInCameraFrame.push_back(yaw-this->camera_yaw);
            objectPoses.push_back(objectPoseInCameraFrame);
          }
        }

        std::vector<std::vector<double>> bounding_box_data;
          for(int i = 0; i< static_cast<int>(classifications.size());i++){
            std::vector<double> bounding_box_for_object = getBoundingBoxData(classifications.at(i),cornerDataForObjects.at(i));
            if(bounding_box_for_object.size()>0){
              if(addPoseToBoundingBoxMessage){
              bounding_box_for_object.push_back(objectPoses.at(i).at(0));
              bounding_box_for_object.push_back(objectPoses.at(i).at(1));
              bounding_box_for_object.push_back(objectPoses.at(i).at(2));
              bounding_box_for_object.push_back(objectPoses.at(i).at(3));
              bounding_box_for_object.push_back(objectPoses.at(i).at(4));
              bounding_box_for_object.push_back(objectPoses.at(i).at(5));
              }
              bounding_box_data.push_back(bounding_box_for_object);
            }
        }
        std_msgs::Float64MultiArray bounding_box_message;
        std::vector<double> bounding_box_message_data;
        lastBoundingBoxData.clear();
        for(int i =0; i<static_cast<int>(bounding_box_data.size());i++){
          for(int j =0; j<static_cast<int>(bounding_box_data.at(i).size());j++){
            bounding_box_message_data.push_back(bounding_box_data.at(i).at(j));
            lastBoundingBoxData.push_back(bounding_box_data.at(i).at(j));
          }
        }
        bounding_box_message.data = bounding_box_message_data;
        //bounding_box_publisher.publish(bounding_box_message);
      }
      std::vector<double> getBoundingBoxData(int classification, std::vector<Vector3d> corners){
        std::vector<double> bounding_box_data;
        std::vector<Vector3d> corners_in_camera_reference_frame;
        for(int i=0;i<static_cast<int>(corners.size());i++){
          Vector3d translatedCorner = rotate(translate(corners.at(i),-this->camera_x,-this->camera_y,-this->camera_z),-this->camera_yaw,-this->camera_pitch,-this->camera_roll);
          if(translatedCorner.x() >0){
          corners_in_camera_reference_frame.push_back(translatedCorner);
          }
        }
        if(corners_in_camera_reference_frame.size()>0){
          bool coordinatePointInFrame = false;
          double minX=1280,minY=720,maxX=0,maxY=0;
          //TESTING DATA
          this->cv_data_points.clear();
          //END TESTING DATA
          for(int i =0;i<static_cast<int>(corners_in_camera_reference_frame.size());i++){
            Vector2d coordinate_in_image_frame = point_in_image_plane(corners_in_camera_reference_frame.at(i));
            //TESTING DATA
            this->cv_data_points.push_back(coordinate_in_image_frame);
            //END TESTIND DATA
            if((0<=coordinate_in_image_frame.x()&&coordinate_in_image_frame.x()<=this->camera_pixel_width) && (0<=coordinate_in_image_frame.y()&&coordinate_in_image_frame.y()<=this->camera_pixel_height)){
              coordinatePointInFrame = true;
            }
            if(coordinate_in_image_frame.x()<minX && coordinate_in_image_frame.y()>=0 && coordinate_in_image_frame.y()<=this->camera_pixel_height){ //&& coordinate_in_image_frame.y()>=0 && coordinate_in_image_frame.y()<=this->camera_pixel_height
              minX = coordinate_in_image_frame.x();
            }
            if(coordinate_in_image_frame.x()>maxX && coordinate_in_image_frame.y()>=0 && coordinate_in_image_frame.y()<=this->camera_pixel_height){
              maxX = coordinate_in_image_frame.x();
            }
            if(coordinate_in_image_frame.y()<minY && coordinate_in_image_frame.x()>=0 && coordinate_in_image_frame.x()<=this->camera_pixel_width){ //&& coordinate_in_image_frame.x()>=0 && coordinate_in_image_frame.x()<=this->camera_pixel_width 
              minY = coordinate_in_image_frame.y();
            }
            if(coordinate_in_image_frame.y()>maxY && coordinate_in_image_frame.x()>=0 && coordinate_in_image_frame.x()<=this->camera_pixel_width){
              maxY = coordinate_in_image_frame.y();
            }
          }
          //NEXT 5 LINES IS AN OPTIONAL SETTING THAT MAKES AT LEAST 1/8 OF THE IMAGE SHOW UP BEFORE IT IS LABELED WITH A BOUNDING BOX
          double ratio_x = abs((std::min(maxX,(double)this->camera_pixel_width)-std::max(minX,0.0)) / (maxX-minX));
          double ratio_y = abs((std::min(maxY,(double)this->camera_pixel_height)-std::max(minY,0.0)) / (maxY-minY));
          if(ratio_x <0.125 || ratio_y<0.125){
            coordinatePointInFrame = false;
          }

          if(coordinatePointInFrame){
            bounding_box_data.push_back(classification);
            bounding_box_data.push_back(std::max(minX,0.0));
            bounding_box_data.push_back(std::min(maxX,(double)this->camera_pixel_width));
            bounding_box_data.push_back(std::max(minY,0.0));
            bounding_box_data.push_back(std::min(maxY,(double)this->camera_pixel_height));
          }
        }
        return bounding_box_data;
      }
      Vector2d point_in_image_plane(Vector3d point){
        Vector2d pixel_coordinates((this->cameraHorizontalFocalLength * point.y() * -1 / point.x())+this->camera_pixel_width/2,(this->cameraVerticalFocalLength * point.z() / point.x())+this->camera_pixel_height/2 );
        return pixel_coordinates;
      }
      Vector3d rotateYaw(Vector3d point, double angle){
        Matrix3d rotationMatrix;
        rotationMatrix(0,0) = cos(angle);
        rotationMatrix(0,1) = -sin(angle);
        rotationMatrix(0,2) = 0;
        rotationMatrix(1,0) = sin(angle);
        rotationMatrix(1,1) = cos(angle);
        rotationMatrix(1,2) = 0;
        rotationMatrix(2,0) = 0;
        rotationMatrix(2,1) = 0;
        rotationMatrix(2,2) = 1;
        return rotationMatrix * point;
      }
      Vector3d rotatePitch(Vector3d point, double angle){
        Matrix3d rotationMatrix;
        rotationMatrix(0,0) = cos(angle);
        rotationMatrix(0,1) = 0;
        rotationMatrix(0,2) = sin(angle);
        rotationMatrix(1,0) = 0;
        rotationMatrix(1,1) = 1;
        rotationMatrix(1,2) = 0;
        rotationMatrix(2,0) = -sin(angle);
        rotationMatrix(2,1) = 0;
        rotationMatrix(2,2) = cos(angle);
        return rotationMatrix * point;
      }
      Vector3d rotateRoll(Vector3d point, double angle){
        Matrix3d rotationMatrix;
        rotationMatrix(0,0) = 1;
        rotationMatrix(0,1) = 0;
        rotationMatrix(0,2) = 0;
        rotationMatrix(1,0) = 0;
        rotationMatrix(1,1) = cos(angle);
        rotationMatrix(1,2) = -sin(angle);
        rotationMatrix(2,0) = 0;
        rotationMatrix(2,1) = sin(angle);
        rotationMatrix(2,2) = cos(angle);
        return rotationMatrix * point;
      }
      Vector3d rotate(Vector3d point, double yaw, double pitch, double roll){
        return rotateYaw(rotatePitch(rotateRoll(point,roll),pitch),yaw);
      }
      Vector3d translate(Vector3d point, double x, double y, double z){
        Vector3d translatedPoint(point.x()+x, point.y()+y, point.z()+z);
        return translatedPoint;
      }
      std::vector<Vector3d> getCornersInWorldFrame(string model_name,double x, double y, double z, double roll, double pitch, double yaw){
        std::vector<Vector3d> corners = object_data_finder.corners_for_model(model_name);
        for(int i =0; i<static_cast<int>(corners.size());i++){
          corners.at(i) = translate(rotate(corners.at(i),yaw,pitch,roll),x,y,z);
        }
        return corners;
      }
      void model_state_callback(const gazebo_msgs::ModelStatesConstPtr& msg){
        if(msg->pose.size()>0){
          for(int i = 0; i < static_cast<int>(msg->name.size());i++){
            if(msg->name.at(i)==car_name){
              geometry_msgs::Quaternion orientation = msg->pose[i].orientation;
              tf2::Quaternion quaternion(orientation.x,orientation.y,orientation.z,orientation.w);
              tf2::Matrix3x3 m(quaternion);
              double roll, pitch, yaw;
              m.getRPY(roll, pitch, yaw);
              geometry_msgs::Point position = msg->pose[i].position;
              this->camera_yaw = yaw;
              this->camera_pitch = pitch;
              this->camera_roll = roll;
              double carX = position.x;
              double carY = position.y;
              double carZ = position.z;
              Vector3d camera_in_car_frame(this->camera_x_in_car_frame,this->camera_y_in_car_frame,this->camera_z_in_car_frame);
              Vector3d camera_in_world_frame = translate(rotate(camera_in_car_frame,yaw,pitch,roll),carX,carY,carZ);
              this->camera_x = camera_in_world_frame.x();
              this->camera_y = camera_in_world_frame.y();
              this->camera_z = camera_in_world_frame.z();
              find_bounding_boxes(msg->pose,msg->name);
            }
            if(msg->name.at(i) == moving_car_name){
              this->moving_car_orientation = msg->pose[i].orientation;
              this->moving_car_position = msg->pose[i].position;
            }
          }
        }
      }
      void recordData(cv::Mat image){
            cv::Mat resizedImage = image;
            //cv::resize(image,resizedImage,cv::Size(desired_x,desired_y));

            std::vector<double> instance_bounding_box = lastBoundingBoxData;
            std::vector<std::vector<double>> bounding_boxes;
            int variable_index = 5 + (int)(include_pose) * 6;

            for(int i=0;i<static_cast<int>(instance_bounding_box.size());i+=variable_index){
              std::vector<double> bounding_box;
              bounding_box.push_back(instance_bounding_box.at(i+0));
              bounding_box.push_back(lastBoundingBoxData.at(i+1)*desired_x/this->camera_pixel_width);
              bounding_box.push_back(lastBoundingBoxData.at(i+2)*desired_x/this->camera_pixel_width);
              bounding_box.push_back((this->camera_pixel_height-lastBoundingBoxData.at(i+3))*desired_y/this->camera_pixel_height);
              bounding_box.push_back((this->camera_pixel_height-lastBoundingBoxData.at(i+4))*desired_y/this->camera_pixel_height);
              if(include_pose){
                bounding_box.push_back(instance_bounding_box.at(i+5));
                bounding_box.push_back(instance_bounding_box.at(i+6));
                bounding_box.push_back(instance_bounding_box.at(i+7));
                bounding_box.push_back(instance_bounding_box.at(i+8));
                bounding_box.push_back(instance_bounding_box.at(i+9));
                bounding_box.push_back(instance_bounding_box.at(i+10));
              }
              bounding_boxes.push_back(bounding_box);
            }
            for(int i=0;i<possible_bounding_boxes_for_image;i++){
              if(i<static_cast<int>(bounding_boxes.size())){
                boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(0)), sizeof(bounding_boxes.at(i).at(0)) );
                boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(1)), sizeof(bounding_boxes.at(i).at(0)) );
                boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(2)), sizeof(bounding_boxes.at(i).at(0)) );
                boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(3)), sizeof(bounding_boxes.at(i).at(0)) );
                boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(4)), sizeof(bounding_boxes.at(i).at(0)) );
                if(include_pose){
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(5)), sizeof(bounding_boxes.at(i).at(0)) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(6)), sizeof(bounding_boxes.at(i).at(0)) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(7)), sizeof(bounding_boxes.at(i).at(0)) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(8)), sizeof(bounding_boxes.at(i).at(0)) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(9)), sizeof(bounding_boxes.at(i).at(0)) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&bounding_boxes.at(i).at(10)), sizeof(bounding_boxes.at(i).at(0)) );
                }

              }else{
                double empty = -1;
                  boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  if(include_pose){
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                    boundingBoxFile.write(reinterpret_cast<char*>(&empty), sizeof(empty) );
                  }
              }
            }
              cv::imwrite(("/home/ryan/TrainingData/object_detection/photos/"+to_string(boundingBoxFileCount) + "-depth-image.TIFF"),resizedImage);
              boundingBoxFileCount += 1;
      }
};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "recorder12354");
    ObjectTracking rw;
    ros::spin();
    return 0;
}
