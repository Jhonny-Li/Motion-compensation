#include <ros/ros.h>
#include <algorithm>
#include <cmath>
#include <cfloat>

#include <iostream>
#include <fstream>

#include <cv_bridge/cv_bridge.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h> 

#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <sensor_msgs/Imu.h>

// #include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include "time.h"
#include <boost/thread.hpp>     
#include <mutex> 

using namespace std;

typedef long long int sll;

std::vector<dvs_msgs::Event> event_buffer;
std::vector<sensor_msgs::Imu> imu_buffer;
std::vector<sensor_msgs::Imu> imu_buffer_;
bool first_event_received = true;

//input params
int height_;
int weight_;
float Focus_;
float pixel_size_;


//main class
class EventVisualizer{
        protected:
                 ros::NodeHandle n_;
                 //publish topic
                 ros::Publisher  image_pub;

                 //subscribe topic
                 ros::Subscriber event_sub;
                 ros::Subscriber imu_sub;

                 std::mutex mtx;
        public:
                 //construct function
                 EventVisualizer (ros::NodeHandle n):
                        n_(n){
                                //sub
                                this->event_sub = this->n_.subscribe("/dvs/events", 1, &EventVisualizer::event_cb, this);
                                this->imu_sub = this->n_.subscribe("/dvs/imu", 7, &EventVisualizer::imu_cb, this);
                                //pub
                                this->image_pub = n_.advertise<sensor_msgs::Image> ("/count_image", 1);
                        }
                 
                 //visualize
                 void show_count_image(std::vector<std::vector<int>>&count_image,int& max_count);

                 //main process function
                 void data_process();
                 
                 //event cb
                 void event_cb(const dvs_msgs::EventArray::ConstPtr& msg){
                        if (first_event_received == false) {

                                mtx.lock();
                                imu_buffer_ = imu_buffer;
                                if(imu_buffer.size() != 0){
                                    imu_buffer.clear();
                                }
                                mtx.unlock();

                                if(imu_buffer_.size() == 0){
                                       return;
                                }
                    
                                for (int i = 0; i < msg->events.size(); ++i) {
                                         event_buffer.emplace_back(msg->events[i]);
                                        
                                }

                                //core function
                                data_process();
                        }else{
                                first_event_received = false;//Discard the first set of data to complete data matching
                                
                                if(imu_buffer.size() != 0){
                                      imu_buffer.clear();  
                                }

                                std::cout<<"Data aligned!"<<std::endl;
                                std::cout<<"Start processing data..."<<std::endl;
                        }
                }

                //imu cb
                 void imu_cb(const sensor_msgs::ImuConstPtr& imu){
                         if(first_event_received == false){
                                 imu_buffer.emplace_back(*imu);        
                        }
                }
}; 


//Show compensated count image 
void EventVisualizer::show_count_image(std::vector<std::vector<int>>&count_image, int& max_count){
        using namespace cv;
        cv::Mat image(height_,weight_,CV_8UC1);
        int scale = (int)(255/max_count) + 1;
        for(int i = 0;i < height_;++i){
                for(int j = 0; j < weight_;++j){
                        image.at<uchar>(i,j) = count_image[i][j]*scale;
                }
        }

        //Change to sensor_message
        sensor_msgs::ImagePtr msg2 = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        image_pub.publish(*msg2); 
    }


void EventVisualizer::data_process(){
         if(imu_buffer_[imu_buffer_.size() - 1].header.stamp.toNSec() > event_buffer[0].ts.toNSec()){
                 
                 float angular_velocity_x=0.0, angular_velocity_y=0.0,angular_velocity_z=0.0;
                 float average_angular_rate_x, average_angular_rate_y,average_angular_rate_z;

                 
                 int cnt=0;//imu counter
                 for(int i=0;i<imu_buffer_.size();++i){
                         if(imu_buffer_[i].header.stamp.toNSec() >= (event_buffer[0].ts.toNSec()-3000000)){
                                 angular_velocity_x+=imu_buffer_[i].angular_velocity.x;
                                 angular_velocity_y+=imu_buffer_[i].angular_velocity.y;
                                 angular_velocity_z+=imu_buffer_[i].angular_velocity.z;
                                 cnt++;
                        }  
                }
                 //Calculate the average imu angular rates
                 average_angular_rate_x = angular_velocity_x/float(cnt);
                 average_angular_rate_y = angular_velocity_y/float(cnt);
                 average_angular_rate_z = angular_velocity_z/float(cnt);
                 float average_angular_rate = std::sqrt((average_angular_rate_x*average_angular_rate_x) + (average_angular_rate_y*average_angular_rate_y) + (average_angular_rate_z*average_angular_rate_z));

                 
                 //Motion  compensation
                 sll t0=event_buffer[0].ts.toNSec();//the first event
                 float time_diff = 0.0;//time diff
                 std::vector<std::vector<int>>count_image(height_,std::vector<int>(weight_));//count image
                 std::vector<std::vector<float>>time_image(height_,std::vector<float>(weight_));//time image
                 for(int i=0;i<event_buffer.size();++i){
                        time_diff = double(event_buffer[i].ts.toNSec()-t0)/1000000000.0;

                        //Calculate the rotation offset of the event point
                        float x_angular=time_diff*average_angular_rate_x;
                        float y_angular=time_diff*average_angular_rate_y;
                        float z_angular=time_diff*average_angular_rate_z;

                        
                        int x=event_buffer[i].x - weight_/2; 
                        int y=event_buffer[i].y - height_/2;
                        
                        //Angle of initial position of event point
                        float pre_x_angel = atan(y*pixel_size_/Focus_);
                        float pre_y_angel = atan(x*pixel_size_/Focus_);

                        //compensate
                        int compen_x = (int)((x*cos(z_angular) - sin(z_angular)*y) - (x - (Focus_*tan(pre_y_angel + y_angular)/pixel_size_)) + weight_/2);
                        int compen_y = (int)((x*sin(z_angular) + cos(z_angular)*y) - (y - (Focus_*tan(pre_x_angel - x_angular)/pixel_size_)) + height_/2);
                        event_buffer[i].x = compen_x;
                        event_buffer[i].y = compen_y;
                        
                        
                        //count image and time image
                        if(compen_y < height_ && compen_y >= 0 && compen_x < weight_ && compen_x >= 0){
                            if(count_image[compen_y][compen_x]<20)count_image[compen_y][compen_x]++; 
                            time_image[compen_y][compen_x] += time_diff;
                        }
                    }

                 int max_count = 0;
                 float max_time = 0.0;
                 float total_time = 0.0;
                 float average_time = 0.0;
                 int trigger_pixels = 0;

                 for(int i = 0; i<height_; ++i){
                        for(int j = 0; j < weight_; ++j){
                                if(count_image[i][j] != 0){
                                        time_image[i][j] /= count_image[i][j];
                                        max_count = std::max(max_count,count_image[i][j]);
                                }
                        }
                 }

                  
                 //Visualize compensated count image
                 show_count_image(count_image, max_count);  
                
                 //release buffer
                 event_buffer.clear();
                 imu_buffer_.clear();
        }else{
                 //keep safe 
                if(event_buffer.size() != 0){

                        event_buffer.clear();
                } 
                if(imu_buffer_.size() != 0){

                        imu_buffer_.clear();
                }
        }
}

//main
  int main(int argc, char** argv)
   {
   ros::init(argc, argv, "datasync_node");
 
   ros::NodeHandle nh;
   EventVisualizer visualizer(nh);
   ros::NodeHandle nh_priv("~");

   nh_priv.param<int>("weight_param", weight_, 346);
   nh_priv.param<int>("height_param", height_, 260);
   nh_priv.param<float>("focus", Focus_, 6550);
   nh_priv.param<float>("pixel_size",pixel_size_ , 18.5);


   ros::AsyncSpinner spinner(3); // Use 3 threads
   spinner.start();
   ros::waitForShutdown();

   return 0;
 }
