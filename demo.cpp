/* Looked example from:
http://www.microhowto.info/howto/listen_for_and_receive_udp_datagrams_in_c.html
*/


#include <iostream>
#include <errno.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <bitset>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher()
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("my_rsbp_topic",1);
      message.height = 1;
      message.width = 384;
      message.point_step = 6;

      sensor_msgs::PointCloud2Modifier modifier(message);
      modifier.setPointCloud2Fields(4, "azimuth", 1, sensor_msgs::msg::PointField::UINT16,
                                     "elevation", 1, sensor_msgs::msg::PointField::UINT8,
                                     "distance", 1, sensor_msgs::msg::PointField::UINT16,
                                     "reflectivity", 1, sensor_msgs::msg::PointField::UINT8);

      modifier.resize(384);
    }

    sensor_msgs::msg::PointCloud2 message;
    std::shared_ptr<sensor_msgs::msg::PointCloud2> message_ptr;

    void publish_pointcloud()
    {
      //message.data = "Hello, world! " + std::to_string(count_++);
      //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      //(std::shared_ptr< const MessageT >
      publisher_->publish(message);
    }
  private:

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    size_t count_;
  };



#define PRINT_STUFF

int main(int argc, char * argv[]){
  std::cout << "This is a very primitive Robosense MSOP parser and publisher" << std::endl;

  // Initializing UDP listening
  const char* hostname=0; /* wildcard */
  const char* portname="6699";
  struct addrinfo hints;
  memset(&hints,0,sizeof(hints));
  hints.ai_family=AF_INET;
  hints.ai_socktype=SOCK_DGRAM;
  hints.ai_protocol=IPPROTO_UDP;
  hints.ai_flags=AI_PASSIVE;
  struct addrinfo* res=0;
  int err=getaddrinfo(hostname,portname,&hints,&res);
  if (err!=0) {
    fprintf(stderr, "failed to resolve local socket address (err=%d)\n",err);
    return -1;
  }

  int fd=socket(res->ai_family,res->ai_socktype,res->ai_protocol);
  if (fd==-1) {
    fprintf(stderr, "socket error: %s\n",strerror(errno));
    return -1;
  }

  if (bind(fd,res->ai_addr,res->ai_addrlen)==-1) {
    fprintf(stderr, "bind error: %s\n",strerror(errno));
    return -1;
  }
  freeaddrinfo(res);

  // ROS2 initialization stuff
  // https://answers.ros.org/question/338026/zero-latency-publishing-of-sensor-data/
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("IMU_node");
  MinimalPublisher my_publisher;
  //rclcpp::Node rsbp_node("my_node");
  //auto rsbp_node = rclcpp::Node::make_shared("my_node");
  //auto rsbp_pub = rsbp_node->create_publisher<sensor_msgs::msg::PointCloud2>("my_rsbp_topic");
  //auto rsbp_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();




  // Main loop
  while(1){

    char buffer[2048];
    struct sockaddr_storage src_addr;
    socklen_t src_addr_len=sizeof(src_addr);
    ssize_t count=recvfrom(fd,buffer,sizeof(buffer),0,(struct sockaddr*)&src_addr,&src_addr_len);
    if (count==-1) {
      fprintf(stderr, "recvfrom error: %s\n",strerror(errno));
      return -1;
    }
    else if (count==sizeof(buffer)) {
      fprintf(stderr, "datagram too large for buffer: truncated\n");
    }




// MAIN PART of the loop
// MSOP protocol information can be found in robosense manuals
    else if(count==1248){
      #ifdef PRINT_STUFF
      std::cout << std::endl;
      printf("Robosense MSOP message received, length %d \n", (int) count);
      #endif
      
      int msg_offset = 42;
      
      my_publisher.publish_pointcloud();
	}






    else{
      #ifdef PRINT_STUFF
      std::cout << std::endl;
      printf("UDP message received, length %d \n", (int) count);
      #endif
	}
    }
}
