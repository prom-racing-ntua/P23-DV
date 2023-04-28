//#define PRINT_FLAG 

#include <fstream>
#include <memory>
#include <chrono>
#include <string>
#include <functional>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <image_transport/image_transport.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "hesai_lidar/msg/pandar_scan.hpp"
#include "hesai_lidar/msg/pandar_packet.hpp"

// using namespace std;

namespace ns_hesai_lidar
{
class HesaiLidarClient: public rclcpp::Node {
public:
	HesaiLidarClient():Node("hesai_lidar") {
		setParameters();

		rclcpp::QoS qos(rclcpp::KeepLast(7));
		lidarPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar", rclcpp::SensorDataQoS());
		packetPublisher = this->create_publisher<hesai_lidar::msg::PandarScan>("pandar_packets", qos);

		this->timerCallback();

		RCLCPP_WARN(get_logger(), "HesaiLidarClient Initialized");
	}


private:
	PandarGeneralSDK* hsdk;

	std::string m_sPublishType;
	std::string m_sTimestampType;

	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPublisher;
	rclcpp::Publisher<hesai_lidar::msg::PandarScan>::SharedPtr packetPublisher;

	rclcpp::Subscription<hesai_lidar::msg::PandarScan>::SharedPtr packetSubscriber;

	rclcpp::TimerBase::SharedPtr timer_;


	void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::msg::PandarScan::SharedPtr scan) // the timestamp from first point cloud of cld
	{
		RCLCPP_INFO(get_logger(), "Entered lidarCallback");

		if (m_sPublishType == "both" || m_sPublishType == "points")
		{
			// pcl_conversions::toPCL(rclcpp::Time(timestamp), cld->header.stamp);
			sensor_msgs::msg::PointCloud2 output;
			pcl::toROSMsg(*cld, output);
			output.header.stamp = now();
			lidarPublisher->publish(output);
#ifdef PRINT_FLAG
			std::cout.setf(ios::fixed);
			std::cout << "timestamp: " << std::setprecision(10) << timestamp << ", point size: " << cld->points.size() << std::endl;
#endif        
		}
		if (m_sPublishType == "both" || m_sPublishType == "raw")
		{
			packetPublisher->publish(*scan);
#ifdef PRINT_FLAG
			std::cout << "raw size: " << scan->packets.size() << std::endl;
#endif
		}
	}


	void gpsCallback(int timestamp) {
#ifdef PRINT_FLAG
		std::cout << "gps: " << timestamp << std::endl;
#endif      
	}


	void scanCallback(const hesai_lidar::msg::PandarScan::SharedPtr scan) {
		hsdk->PushScanPacket(scan);
	}


	void timerCallback() {
		std::string serverIp;
		int lidarRecvPort;
		int gpsPort;
		double startAngle;
		std::string lidarCorrectionFile;  // Get local correction when getting from lidar failed
		std::string lidarType;
		std::string frameId;
		int pclDataType;
		std::string pcapFile;
		std::string dataType;
		std::string multicastIp;
		bool coordinateCorrectionFlag;
		std::string targetFrame;
		std::string fixedFrame;

		this->get_parameter("pcap_file", pcapFile);
		this->get_parameter("server_ip", serverIp);
		this->get_parameter("lidar_recv_port", lidarRecvPort);
		this->get_parameter("gps_port", gpsPort);
		this->get_parameter("start_angle", startAngle);
		this->get_parameter("lidar_correction_file", lidarCorrectionFile);
		this->get_parameter("lidar_type", lidarType);
		this->get_parameter("frame_id", frameId);
		this->get_parameter("pcldata_type", pclDataType);
		this->get_parameter("publish_type", m_sPublishType);
		this->get_parameter("timestamp_type", m_sTimestampType);
		this->get_parameter("data_type", dataType);
		this->get_parameter("multicast_ip", multicastIp);
		this->get_parameter("coordinate_correction_flag", coordinateCorrectionFlag);
		this->get_parameter("target_frame", targetFrame);
		this->get_parameter("fixed_frame", fixedFrame);
		this->get_parameter("background_b", targetFrame);

		if (!pcapFile.empty())
		{
			std::string pcap_file_path{ament_index_cpp::get_package_share_directory("hesai_lidar") + '/' + pcapFile};
			RCLCPP_WARN_STREAM(get_logger(), "PCAP file set to: " << pcap_file_path);

			hsdk = new PandarGeneralSDK(pcap_file_path, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
				static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, lidarCorrectionFile, \
				coordinateCorrectionFlag, targetFrame, fixedFrame);

			if (hsdk != NULL)
			{
				std::ifstream fin(lidarCorrectionFile);
				if (fin.is_open())
				{
					std::cout << "Open correction file " << lidarCorrectionFile << " succeed" << std::endl;
					int length = 0;
					std::string strlidarCalibration;
					fin.seekg(0, std::ios::end);
					length = fin.tellg();
					fin.seekg(0, std::ios::beg);
					char* buffer = new char[length];
					fin.read(buffer, length);
					fin.close();
					strlidarCalibration = buffer;
					int ret = hsdk->LoadLidarCorrectionFile(strlidarCalibration);
					if (ret != 0)
					{
						std::cout << "Load correction file from " << lidarCorrectionFile << " failed" << std::endl;
					}
					else
					{
						std::cout << "Load correction file from " << lidarCorrectionFile << " succeed" << std::endl;
					}
				}
				else
				{
					std::cout << "Open correction file " << lidarCorrectionFile << " failed" << std::endl;
				}
			}
		}
		else if ("rosbag" == dataType)
		{
			hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
				static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType, \
				lidarCorrectionFile, coordinateCorrectionFlag, targetFrame, fixedFrame);
			if (hsdk != NULL)
			{
				packetSubscriber = this->create_subscription<hesai_lidar::msg::PandarScan>("pandar_packets", 10, std::bind(&HesaiLidarClient::scanCallback, this, std::placeholders::_1));
			}
		}
		else
		{
			RCLCPP_WARN(get_logger(), "Connecting to LiDAR Device");
			hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
				boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
				boost::bind(&HesaiLidarClient::gpsCallback, this, _1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, \
				m_sTimestampType, lidarCorrectionFile, multicastIp, coordinateCorrectionFlag, targetFrame, fixedFrame);
		}

		if (hsdk != NULL)
		{
			hsdk->Start();
			// hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
		}
		else
		{
			printf("create sdk fail\n");
		}
	}


	void setParameters() {
		this->declare_parameter<std::string>("pcap_file", "");
		this->declare_parameter<std::string>("server_ip", "");
		this->declare_parameter<int>("lidar_recv_port", 2368);
		this->declare_parameter<int>("gps_port", 10110);
		this->declare_parameter<double>("start_angle", 0.0);
		this->declare_parameter<std::string>("lidar_correction_file", "");
		this->declare_parameter<std::string>("lidar_type", "");
		this->declare_parameter<std::string>("frame_id", "");
		this->declare_parameter<int>("pcldata_type", 0);
		this->declare_parameter<std::string>("publish_type", "");
		this->declare_parameter<std::string>("timestamp_type", "");
		this->declare_parameter<std::string>("data_type", "");
		this->declare_parameter<std::string>("multicast_ip", "");
		this->declare_parameter<bool>("coordinate_correction_flag", false);
		this->declare_parameter<std::string>("target_frame", "");
		this->declare_parameter<std::string>("fixed_frame", "");
	}
};
} // ns_hesai_lidar



int main(int argc, char** argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ns_hesai_lidar::HesaiLidarClient>());
	rclcpp::shutdown();
	return 0;
}