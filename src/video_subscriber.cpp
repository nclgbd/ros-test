#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <sensor_msgs/image_encodings.h>

class VideoHandler {

    ros::NodeHandle node_handle;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;

public:

    VideoHandler():it(node_handle) {
        sub = it.subscribe("/camera/image_raw", 1, &VideoHandler::get_frame, this);
        pub = it.advertise("/video_publisher/output_video", 1);

    }

    void get_frame(const sensor_msgs::ImageConstPtr& frame) {
        cv_bridge::CvImagePtr cv_ptr;

        try {
            cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
            pub.publish(cv_ptr->toImageMsg());

        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;

        }

    }

};


int main(int argc, char** argv) {

    ros::init(argc, argv, "video_subscriber_node");
    VideoHandler vh;
    ros::spin();
    
}