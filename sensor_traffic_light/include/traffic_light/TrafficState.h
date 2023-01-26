#ifndef TRAFFIC_STATE_H
#define TRAFFIC_STATE_H
#include "ros/console.h"
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include "t3_msgs/BoundingBox.h"
#include "t3_msgs/traffic_light_data.h"
#include "t3_msgs/traffic_light_image.h"
#include <opencv2/opencv.hpp>
#include <vector>

const std::string SUB_TOPIC = "traffic_light_image";
const std::string PUB_TOPIC = "traffic_light_data";

struct BoundingBox 
{
    float probability;
    int xmin;
    int ymin;
    int xmax;
    int ymax;

    BoundingBox()
    :probability(0),xmin(0),ymin(0),xmax(0),ymax(0){};
}

struct Traffic
{
    public:
    
        BoundingBox boundingBox;
        int height;
        int weight;
        int square;
        int center_x;
        int center_y;
        cv::Mat image;
        // RED = 0; YELLOW = 1; GREEN = 2;
        int8_t color_= -1;
        bool detected;
    
    :height(0),weight(0),square(0),center_x(0),center_y(0),color_(-1),detected(false){};
    {
        image = cv::Mat::zeros(352,352,CV_8UC3);
    };

    void update(const t3_msgs::traffic_light_image::ConstPtr& msg)
    {
        image = cv::Mat(352, 352, CV_8UC3, const_cast<uchar*>(&msg->data[0]), msg->step);
        boundingBox = BoundingBox();
        boundingBox.probability = msg->probability;
        boundingBox.xmin = msg->xmin;
        boundingBox.ymin = msg->ymin;
        boundingBox.xmax = msg->xmax;
        boundingBox.ymax = msg->ymax;

        height = boundingBox.ymax - boundingBox.ymin;
        weight = boundingBox.xmax - boundingBox.xmin;
        square = height * weight;
        center_x = (boundingBox.xmax + boundingBox.xmin) / 2;
        center_y = (boundingBox.ymax + boundingBox.ymin) / 2;
    };
};


class Imageprocess{
    public:

        ros::NodeHandle node;
        ros::Subscriber sub;
        ros::Subscriber sub_cam;
        ros::Publisher pub;
        Traffic traffic;
        
        Imageprocess()
        {
            this->sub = node.subscribe(SUB_TOPIC, 1, &Imageprocess::callbackTraffic, this);
            this->pub = node.advertise<t3_msgs::traffic_light_data>(PUB_TOPIC, 1);
        };
        void callbackTraffic(const t3_msgs::traffic_light_image::ConstPtr& msg){
            traffic->update(msg);
        };
        void publish()
        {
            t3_msgs::traffic_light_data msg;
            msg.header.stamp = ros::Time::now();
            msg.detected = traffic.detected;
            msg.color = traffic.color;
            msg.bounding_box = traffic.bounding_box;
            pub.publish(msg);
        };
        void detect_traffic_light()
        {
            cv::Scalar lower_blue (160,50,50);
            cv::Scalar upper_blue (180,255,255);
            cv::Mat roi = traffic.image(
                cv::Range(traffic.center_y-traffic.height/2,traffic.center_y+traffic.height/2),
                cv::Range(traffic.center_x-traffic.weight/4,traffic.center_x+traffic.weight/4)
            );
            
            cv::Mat new_img;
            cv::cvtColor(roi,new_img,cv::COLOR_BGR2HSV);

            cv::Mat hsv_img;
            cv::cvtColor(new_img,hsv_img,cv::COLOR_BGR2HSV);

            cv::Mat mask_red;
            cv::inRange(hsv_img,lower_blue,upper_blue,mask_red);

            cv::Mat mask_img_red;
            cv::bitwise_and(new_img,new_img,mask_img_red,mask_red);

            cv::Mat gray_img;
            cv::cvtColor(mask_img_red,gray_img,cv::COLOR_BGR2GRAY);

            cv::Mat th_img;
            cv::threshold(gray_img,th_img,1,255,cv::THRESH_BINARY_INV);

            int row = static_cast<int>(traffic.height*416*0.3);

            int count1 = 0;
            int count2 = 0;
            int count3 = 0;
            for (int i = 0; i< row; i++){
                for (int j = 0; j< traffic.width_; j++){
                    if (th_img.at<int>(i,j)==0){
                        count1++;
                    }
                    if(th_img.at<int>(i+static_cast<int>(0.35*traffic.height),j)==0){
                        count2++;
                    }
                    if(th_img.at<int>(i+static_cast<int>(0.65*traffic.height),j)==0){
                        count3++;
                    }
                }
            }
            int decision1 = static_cast<int>(count1/traffic.square*1000);
            int decision2 = static_cast<int>(count2/traffic.square*1000);
            int decision3 = static_cast<int>(count3/traffic.square*1000);

            int min_ = std::min(decision1, decision3);
            int max = std::max(decision1, decision3);

            // RED = 0; YELLOW = 1; GREEN = 2;
            if(decision2>100)
            {
                // Yellow
                traffic.color_ = 1;
            }else if (max>30 && max < 60)
            {
                // Red
                traffic.color_ = 0;
            }else if (max>100)
            {
                // Green
                traffic.color_ = 0;
            }else
            {
                // Turn Off
                traffic.color_ = -1;
                traffic.detected = false;
            }
            publish();
        };
};

#endif