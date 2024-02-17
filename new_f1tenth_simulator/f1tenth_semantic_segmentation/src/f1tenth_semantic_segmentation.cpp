#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;
// //Window used for testing
// static const std::string TEST_WINDOW = "edge window";

//int rmin = 0,rmax = 255,gmin=0, gmax=255,bmin=0,bmax=255;
class SemanticSegmentation{
    private:
    ros::NodeHandle n;
    image_transport::ImageTransport image_transport;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    int outputX = 256, outputY=256;
    int inputX = 1280, inputY = 720;
    bool fastMode;
    bool visualize_topic = false;
    public:
    SemanticSegmentation():image_transport(n) {
        n = ros::NodeHandle("~");
         n.getParam("/output_height",outputY);
         n.getParam("/output_width",outputX);
         n.getParam("/input_height",inputY);
         n.getParam("/input_width",inputX);
         std::string camera_topic, publish_topic;
         n.getParam("camera_topic",camera_topic);
        image_sub_ = image_transport.subscribe(camera_topic, 1, &SemanticSegmentation::image_callback, this);
         n.getParam("publish_topic", publish_topic);
        image_pub_ = image_transport.advertise(publish_topic, 1);
        n.getParam("/visualize_semantic_segmentation",visualize_topic);
        // // Sliders Used for testing and finding the right rgb range
        // namedWindow(TEST_WINDOW);
        //  namedWindow("Trackbars",(640,200));
        //  createTrackbar("R min","Trackbars",&rmin,255);
        //  createTrackbar("R max","Trackbars",&rmax,255);
        //  createTrackbar("G min","Trackbars",&gmin,255);
        //  createTrackbar("G max","Trackbars",&gmax,255);
        //  createTrackbar("B min","Trackbars",&bmin,255);
        //  createTrackbar("B max","Trackbars",&bmax,255);
    }
    ~SemanticSegmentation()
  {
  }
  Mat getMask(Mat input, int r, int g, int b){
      Scalar lower(r,g,b);
      Scalar upper(r,g,b);
      Mat colorFiltered;
      inRange(input,lower,upper,colorFiltered);
      return colorFiltered;
  }
  void image_callback(const sensor_msgs::ImageConstPtr& msg){
      cv_bridge::CvImagePtr cv_ptr;
      Mat outputMat;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
      Mat imageInRGB;
      cvtColor(cv_ptr->image,imageInRGB,COLOR_BGRA2RGB);
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
      cars = cars * 1 /256.0;
      yellowLaneLines = yellowLaneLines * 2/256.0;
      whiteLaneLines = whiteLaneLines * 3/256.0;

      Mat combinedMat = cars | yellowLaneLines | whiteLaneLines;
      if(visualize_topic){
        combinedMat = combinedMat * 80.0;
      }
      Mat scaledImage;
      cv::resize(combinedMat,scaledImage,Size(outputX,outputY));
      outputMat = combinedMat;
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    //Upload Mat Image as ROS TOPIC 
    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header; 
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; 
    out_msg.image    = outputMat;
    //Output modified video stream
    image_pub_.publish(out_msg.toImageMsg());

    // // Lines to add if calibrating semantic segmentation image
    // cv::imshow(TEST_WINDOW, outputMat);
    // //wait time is in miliseconds
    // cv::waitKey(3);
  }
};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "semantic_segmentation");
    SemanticSegmentation rw;
    ros::spin();
    return 0;
}
