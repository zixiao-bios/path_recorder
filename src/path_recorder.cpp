#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <signal.h>
#include <boost/bind.hpp>

bool ok = true;

class PathRecorder {
public:
  // 构造函数，传入文件路径和路径精度
  PathRecorder(const std::string &file_path, double path_precision)
    : file_path_(file_path), path_precision_(path_precision), first_pose_(true) {
        std::ifstream fis(file_path_);
    if (fis.is_open()) {
        fis.close();
        ROS_ERROR("File [%s] already exists! Please provide a non-existent file.", file_path_.c_str());
        exit(1);
    } else {
        output_file_.open(file_path_, std::ofstream::out);
        if (!output_file_.is_open()) {
            ROS_ERROR("Failed to open output file!");
            exit(1);
        }
    }
  }

  // 析构函数
  ~PathRecorder() {
    if (output_file_.is_open()) {
      output_file_.close();
    }
  }

  // 添加路径点的函数
  bool add_pose(const geometry_msgs::TransformStamped &pose, bool force_add = false) {
    if (first_pose_) {
      write_pose_to_file(pose);
      last_pose_ = pose;
      first_pose_ = false;
      return true;
    }

    double distance = std::sqrt(std::pow(pose.transform.translation.x - last_pose_.transform.translation.x, 2) +
                                std::pow(pose.transform.translation.y - last_pose_.transform.translation.y, 2));

    if (force_add or distance > path_precision_) {
      write_pose_to_file(pose);
      last_pose_ = pose;
      return true;
    } else {
      return false;
    }
  }

private:
  // 将路径点写入文件的函数
  void write_pose_to_file(const geometry_msgs::TransformStamped &pose) {
    if (output_file_.is_open()) {
      output_file_ << pose.header.stamp << " "
                   << pose.transform.translation.x << " "
                   << pose.transform.translation.y << " "
                   << pose.transform.translation.z << " "
                   << pose.transform.rotation.x << " "
                   << pose.transform.rotation.y << " "
                   << pose.transform.rotation.z << " "
                   << pose.transform.rotation.w << std::endl;
    } else {
      ROS_ERROR("Output file is not open!");
    }
  }

  std::string file_path_;
  double path_precision_;
  geometry_msgs::TransformStamped last_pose_;
  bool first_pose_;
  std::ofstream output_file_;
};

void sig_handler(int sig) {
  if (sig == SIGINT) {
    ok = false;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "path_recorder_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh("~");

  // 获取参数
  std::string file_path, global_frame, robot_frame;
  double path_precision;
  nh.param<std::string>("file_path", file_path, "robot_path.txt");
  nh.param<double>("path_precision", path_precision, 0.2);
  nh.param<std::string>("global_frame", global_frame, "map");
  nh.param<std::string>("robot_frame", robot_frame, "base_link");

  // 创建PathRecorder对象
  static PathRecorder path_recorder(file_path, path_precision);

  // 创建tf监听器
  static tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // 绑定信号处理函数
  signal(SIGINT, &sig_handler);

  // 主循环
  ros::Rate rate(10.0);
  geometry_msgs::TransformStamped transform_stamped;

  while (ok) {
    try {
      transform_stamped = tf_buffer.lookupTransform(global_frame, robot_frame, ros::Time(0));
      bool added = path_recorder.add_pose(transform_stamped);
      if (added) {
        ROS_INFO("Pose added.");
      }
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ros::spinOnce();
    rate.sleep();
  }

  // add last pose
  try {
      path_recorder.add_pose(transform_stamped, true);
      ROS_INFO("Last pose added.");
  } catch (tf2::TransformException &ex) {
    ROS_ERROR("Last pose add failed!");
    ROS_WARN("%s", ex.what());
  }

  return 0;
}
