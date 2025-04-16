#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <arpa/inet.h>
#include <unistd.h>

#define PORT 12345
#define BUFFER_SIZE 65535

int main(int argc, char** argv) {
    ros::init(argc, argv, "unilidar_udp_receiver");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("unilidar/points", 1);

    int sockfd;
    struct sockaddr_in servaddr;
    char buffer[BUFFER_SIZE];

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    servaddr.sin_family = AF_INET;
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(PORT);

    bind(sockfd, (const struct sockaddr*)&servaddr, sizeof(servaddr));
    ROS_INFO("Listening for UDP packets on port %d...", PORT);

    ros::Rate rate(30);
    while (ros::ok()) {
        ssize_t len = recv(sockfd, buffer, BUFFER_SIZE, MSG_DONTWAIT);
        if (len > 0) {
            // TODO: Parse actual point cloud from Unitree UDP format
            sensor_msgs::PointCloud2 msg;
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "map";
            msg.height = 1;
            msg.width = 0; // updated after filling

            sensor_msgs::PointCloud2Modifier modifier(msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(100); // dummy size for now

            sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
            for (int i = 0; i < 100; ++i, ++iter_x) {
                iter_x[0] = i * 0.1f;
                iter_x[1] = i * 0.1f;
                iter_x[2] = 0.0f;
                msg.width++;
            }

            pub.publish(msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    close(sockfd);
    return 0;
}

