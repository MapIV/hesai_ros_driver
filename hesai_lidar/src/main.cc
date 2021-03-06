#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

class HesaiLidarClient
{
private:
  ros::Publisher lidar_publisher_, lidar_publisher_ex_;
  ros::Publisher packet_publisher_;
  PandarGeneralSDK *pandar_sdk_ptr_;
  std::string timestamp_type_;
  ros::Subscriber packet_subscriber_;
  bool use_rosbag_;
  int time_shift_threshold_;
  std::string frame_id_;

public:
  HesaiLidarClient()
  {
    ros::NodeHandle private_handle("~");
    ros::NodeHandle global_nh;
    lidar_publisher_ = global_nh.advertise<sensor_msgs::PointCloud2>("points_raw", 10);
    lidar_publisher_ex_ = global_nh.advertise<sensor_msgs::PointCloud2>("points_raw_ex", 10);

    std::string lidar_ip;
    int lidar_recv_port;
    int gps_port;
    double start_angle;
    std::string lidar_correction_file;  // Get local correction when getting from lidar failed
    std::string lidar_type;
    int pcl_data_type;
    std::string pcap_file;

    private_handle.getParam("pcap_file", pcap_file);
    private_handle.getParam("lidar_ip", lidar_ip);
    private_handle.getParam("lidar_recv_port", lidar_recv_port);
    private_handle.getParam("gps_port", gps_port);
    private_handle.getParam("start_angle", start_angle);
    private_handle.getParam("lidar_correction_file", lidar_correction_file);
    private_handle.getParam("lidar_type", lidar_type);
    private_handle.getParam("pcldata_type", pcl_data_type);
    private_handle.getParam("timestamp_type", timestamp_type_);
    private_handle.param<bool>("use_rosbag", use_rosbag_, false);
    private_handle.param<int>("time_shift_threshold", time_shift_threshold_, 10);
    private_handle.param<std::string>("frame_id", frame_id_, lidar_type);

    int time_zone = 0;


    if (!pcap_file.empty())
    {
      pandar_sdk_ptr_ = new PandarGeneralSDK(pcap_file, boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(start_angle * 100 + 0.5), time_zone, pcl_data_type, lidar_type, frame_id_, timestamp_type_);
      if (pandar_sdk_ptr_ != NULL && !lidar_correction_file.empty())
      {
        std::ifstream fin(lidar_correction_file);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        pandar_sdk_ptr_->LoadLidarCorrectionFile(strlidarCalibration);
      }
    }
    if (use_rosbag_)
    {
      ROS_INFO_STREAM("Rosbag");
      packet_subscriber_ = global_nh.subscribe("pandar_packets", 10, &HesaiLidarClient::scanCallback,
                                               (HesaiLidarClient *) this, ros::TransportHints().tcpNoDelay(true));

      pandar_sdk_ptr_ = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
      static_cast<int>(start_angle * 100 + 0.5), time_zone, pcl_data_type, lidar_type, frame_id_, timestamp_type_);
      if (pandar_sdk_ptr_ != NULL && !lidar_correction_file.empty())
      {
        std::ifstream fin(lidar_correction_file);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        pandar_sdk_ptr_->LoadLidarCorrectionFile(strlidarCalibration);
      }
    } else
    {
      packet_publisher_ = global_nh.advertise<hesai_msgs::PandarScan>("pandar_packets", 10);
      pandar_sdk_ptr_ = new PandarGeneralSDK(lidar_ip, lidar_recv_port, gps_port, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3), \
        NULL, static_cast<int>(start_angle * 100 + 0.5), time_zone, pcl_data_type, lidar_type, frame_id_, timestamp_type_);
    }

    if (pandar_sdk_ptr_ != NULL)
    {
      pandar_sdk_ptr_->Start();
    } else
    {
      ROS_ERROR_STREAM(ros::this_node::getName() << "Error while creating Hesai Lidar Object");
    }
  }

  void lidarCallback(boost::shared_ptr<PPointCloudXYZIRADT> cld_ex, double timestamp, hesai_msgs::PandarScanPtr scan)
  {
    const int HOUR_TO_SEC = 3600;
    ros::Time system_time = ros::Time::now(); // use this to recover the hour
    uint32_t cur_hour = system_time.sec / HOUR_TO_SEC;
    ros::Time sensor_time = ros::Time(timestamp);
    ros::Time sensor_time_ok = resolveHourAmbiguity(sensor_time, system_time);

    if (abs(int(sensor_time_ok.toSec()) - int(system_time.toSec())) < time_shift_threshold_)
    {
      pcl_conversions::toPCL(ros::Time(sensor_time_ok), cld_ex->header.stamp);
    } else
    {
      pcl_conversions::toPCL(ros::Time(system_time), cld_ex->header.stamp);
      ROS_WARN_STREAM_THROTTLE(10, ros::this_node::getName()
        << " | Using system time. Large Time difference between device and system. Sensor:"
        << sensor_time_ok << " System:" << system_time << ". Check clock source (GPS/PTP)");
    }

    boost::shared_ptr<PPointCloudXYZIR> out_cld_raw(boost::make_shared<PPointCloudXYZIR>());
    out_cld_raw->points.resize(cld_ex->points.size());
    for (size_t i = 0; i < cld_ex->points.size(); i++) {
      out_cld_raw->points[i].x = cld_ex->points[i].x;
      out_cld_raw->points[i].y = cld_ex->points[i].y;
      out_cld_raw->points[i].z = cld_ex->points[i].z;
      out_cld_raw->points[i].ring = cld_ex->points[i].ring;
      out_cld_raw->points[i].intensity = cld_ex->points[i].intensity;
    }

    //convert to ROS
    boost::shared_ptr<sensor_msgs::PointCloud2> output_ex(boost::make_shared<sensor_msgs::PointCloud2>());
    boost::shared_ptr<sensor_msgs::PointCloud2> output_raw(boost::make_shared<sensor_msgs::PointCloud2>());

    pcl::toROSMsg(*cld_ex, *output_ex);
    pcl::toROSMsg(*out_cld_raw, *output_raw);
    output_raw->header = output_ex->header;
    lidar_publisher_ex_.publish(output_ex);
    lidar_publisher_.publish(output_raw);

    if (!use_rosbag_)
    {
      scan->header = output_ex->header;
      packet_publisher_.publish(scan);
    }

  }

  void scanCallback(const hesai_msgs::PandarScanPtr scan)
  {
    pandar_sdk_ptr_->PushScanPacket(scan);
  }

  ros::Time resolveHourAmbiguity(const ros::Time &stamp, const ros::Time &nominal_stamp)
  {
    const int HALFHOUR_TO_SEC = 1800;
    ros::Time retval = stamp;
    if (nominal_stamp.sec > stamp.sec)
    {
      if (nominal_stamp.sec - stamp.sec > HALFHOUR_TO_SEC)
      {
        retval.sec = retval.sec + 2 * HALFHOUR_TO_SEC;
      }
    } else if (stamp.sec - nominal_stamp.sec > HALFHOUR_TO_SEC)
    {
      retval.sec = retval.sec - 2 * HALFHOUR_TO_SEC;
    }
    return retval;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hesai_pandar");

  HesaiLidarClient pandarClient;

  ros::spin();
  return 0;
}
