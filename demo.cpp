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
#include <cmath>
//#define PRINT_STUFF
#define TRANSFORM_TO_XYZ

#ifdef PRINT_STUFF
const int PRINTMAX = 1;
#endif


class MinimalPublisher : public rclcpp::Node
{
	public:
	MinimalPublisher()
	: Node("minimal_publisher"), count_(0)
	{
		publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("my_rsbp_topic",1);
		#ifndef TRANSFORM_TO_XYZ
		message.height = 1;
		message.width = 384;
		message.point_step = 6;

		sensor_msgs::PointCloud2Modifier modifier(message);
		modifier.setPointCloud2Fields(4, "azimuth", 1, sensor_msgs::msg::PointField::UINT16,
						"elevation", 1, sensor_msgs::msg::PointField::UINT8,
						"distance", 1, sensor_msgs::msg::PointField::UINT16,
						"reflectivity", 1, sensor_msgs::msg::PointField::UINT8);

		modifier.resize(384);
		message.header.frame_id = "rsbp";
		#endif
		#ifdef TRANSFORM_TO_XYZ
		message_xyz.height = 1;
		message_xyz.width = 384;
		message_xyz.point_step = 13;

		sensor_msgs::PointCloud2Modifier modifier(message_xyz);
		modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
						"y", 1, sensor_msgs::msg::PointField::FLOAT32,
						"z", 1, sensor_msgs::msg::PointField::FLOAT32,
						"intensity", 1, sensor_msgs::msg::PointField::UINT8);

		modifier.resize(384);
		message_xyz.header.frame_id = "rsbp_xyz";
		#endif
	}

	sensor_msgs::msg::PointCloud2 message;
	#ifdef TRANSFORM_TO_XYZ
        sensor_msgs::msg::PointCloud2 message_xyz;
	#endif
	std::shared_ptr<sensor_msgs::msg::PointCloud2> message_ptr;

	void publish_pointcloud()
	{
		publisher_->publish(message);

	}
	void publish_xyz_pointcloud()
	{
		publisher_->publish(message_xyz);

	}
	private:

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
		size_t count_;
};


const double elevation_lookuptable[32] = {89.5*(M_PI/180),
					81.0625*(M_PI/180),
					78.25*(M_PI/180),
					72.625*(M_PI/180),
					67*(M_PI/180),
					61.375*(M_PI/180),
					55.75*(M_PI/180),
					50.125*(M_PI/180),
					86.6875*(M_PI/180),
					83.875*(M_PI/180),
					75.4375*(M_PI/180),
					69.8125*(M_PI/180),
					64.1875*(M_PI/180),
					58.5625*(M_PI/180),
					52.9375*(M_PI/180),
					47.3125*(M_PI/180),
					44.5*(M_PI/180),
					38.875*(M_PI/180),
					33.25*(M_PI/180),
					27.625*(M_PI/180),
					22*(M_PI/180),
					16.375*(M_PI/180),
					10.75*(M_PI/180),
					5.125*(M_PI/180),
					41.6875*(M_PI/180),
					36.0625*(M_PI/180),
					30.4375*(M_PI/180),
					24.8125*(M_PI/180),
					19.1875*(M_PI/180),
					13.5625*(M_PI/180)};



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


	int printcounter = 0;
	bool printed_once = false;
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
			if(printcounter < PRINTMAX){
			std::cout << std::endl;
			printf("Robosense MSOP message received, length %d \n", (int) count);
			}
			#endif
      
			int msg_offset = 42;
			int idx_in_msop = 0;
			int azimuth_index = 0;
			int which_point = 0;
			uint8_t azimuth_raw_bytes[2] = {0,0};
			uint16_t azimuth_raw = 0;
			double azimuth = 0.0;
			uint16_t distance = 0;
			uint8_t distance_raw_bytes[2] = {0,0};
			double elevation = 0;
			uint8_t reflectivity = 0;
			float x,y,z = 0;
			#ifdef PRINT_STUFF
			if(!printed_once){
				std::cout << "PRINTING FIRST MESSAGE!" << std::endl << std::endl;
				for(int i = 0; i < 1248; i++){
					printf("%x ", buffer[i]);
					
					
				}
			std::cout << std::endl << std::endl;
			printed_once = true;
			}
			#endif
			for(int n = 0; n < 12; n++){
				// Verify endianness!
				azimuth_index =  msg_offset + 2 + 100*n;
				azimuth_raw_bytes[0] = buffer[azimuth_index + 0];
				azimuth_raw_bytes[1] = buffer[azimuth_index + 1];				
				//memcpy(&azimuth_raw, buffer + msg_offset + 2 + 100*n, 2);

				uint8_t pointcloud2_datapoint_xyz[13];
				// WRONG ENDIANNESS:
				//azimuth_raw = (azimuth_raw_bytes[1] << 8) | (azimuth_raw_bytes[0]);
				// CORRECT ENDIANNESS:
				azimuth_raw = (azimuth_raw_bytes[0] << 8) | azimuth_raw_bytes[1];
				azimuth = double(azimuth_raw)*0.01*(M_PI/180);

 				for(int i = 0; i < 32; i++){

					idx_in_msop = msg_offset + 4 +3*i + 100*n;

					memcpy(&reflectivity, buffer + idx_in_msop + 2, 1);
					distance_raw_bytes[0] = buffer[idx_in_msop + 0];
					distance_raw_bytes[1] = buffer[idx_in_msop + 1];
					distance = (distance_raw_bytes[0] << 8) | (distance_raw_bytes[1]);
					//memcpy(&distance, buffer + idx_in_msop + 0, 2);
					elevation = elevation_lookuptable[i];
					x = float(double(distance)*0.005*cos(elevation)*sin(azimuth));
					y = float(double(distance)*0.005*cos(elevation)*cos(azimuth));
					z = float(double(distance)*0.005*sin(elevation));
					#ifdef PRINT_STUFF
					if(printcounter < PRINTMAX){
						printf("azimuth_raw: %i\n", azimuth_raw);
						printf("azimuth: %f\n", azimuth);
						printf("elevation: %f\n", elevation);
						printf("distance raw: %x\n", distance);
						printf("distance calculated: %f\n", 0.005*distance);
						printf("x: %f\n",x);
						printf("y: %f\n",y);
						printf("z: %f\n",z);
						printcounter++;
					}
					#endif
					memcpy(pointcloud2_datapoint_xyz + 0, &x, sizeof(float));
					memcpy(pointcloud2_datapoint_xyz + 4, &y, sizeof(float));
					memcpy(pointcloud2_datapoint_xyz + 8, &z, sizeof(float));
					memcpy(pointcloud2_datapoint_xyz + 12, &reflectivity, 1);
					// TODO
					for(int j = 0; j < 13; j++){
						my_publisher.message_xyz.data[13*which_point + j] = pointcloud2_datapoint_xyz[j];
					}


					which_point++;
				}
			}
			#ifndef TRANSFORM_TO_XYZ
			my_publisher.publish_pointcloud();
			#else
			my_publisher.publish_xyz_pointcloud();
			#endif
			
		}






		else{
			#ifdef PRINT_STUFF
			std::cout << std::endl;
			printf("UDP message received, length %d \n", (int) count);
			#endif
		}
	}
}
