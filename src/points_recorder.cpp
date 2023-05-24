#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <mutex>
#include <thread>

bool save_point = false;
std::mutex mutex;

class PointRecorder
{
public:
    // 构造函数，传入文件路径
    PointRecorder(const std::string &file_path)
        : file_path_(file_path)
    {
        std::ifstream fis(file_path_);
        if (fis.is_open())
        {
            fis.close();
            ROS_ERROR("File [%s] already exists! Please provide a non-existent file.", file_path_.c_str());
            exit(1);
        }
        else
        {
            output_file_.open(file_path_, std::ofstream::out);
            if (!output_file_.is_open())
            {
                ROS_ERROR("Failed to open output file!");
                exit(1);
            }
        }
    }

    // 析构函数
    ~PointRecorder()
    {
        if (output_file_.is_open())
        {
            output_file_.close();
        }
    }

    // 添加点的函数，返回距离上一个点的距离
    bool add_pose(const geometry_msgs::TransformStamped &pose)
    {
        return write_pose_to_file(pose);
    }

private:
    // 将路径点写入文件的函数
    bool write_pose_to_file(const geometry_msgs::TransformStamped &pose)
    {
        if (output_file_.is_open())
        {
            output_file_ << pose.header.stamp << " "
                         << pose.transform.translation.x << " "
                         << pose.transform.translation.y << " "
                         << pose.transform.translation.z << " "
                         << pose.transform.rotation.x << " "
                         << pose.transform.rotation.y << " "
                         << pose.transform.rotation.z << " "
                         << pose.transform.rotation.w << std::endl;
            return true;
        }
        else
        {
            ROS_ERROR("Output file is not open!");
            return false;
        }
    }

    std::string file_path_;
    std::ofstream output_file_;
};

void input_detect(const ros::NodeHandle &nh)
{
    std::string s;
    while (nh.ok())
    {
        std::getline(std::cin, s);
        mutex.lock();
        save_point = true;
        mutex.unlock();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "point_recorder_node");
    ros::NodeHandle nh("~");

    // 获取参数
    std::string file_path, global_frame, robot_frame;
    nh.param<std::string>("file_path", file_path, "path_list.txt");
    nh.param<std::string>("global_frame", global_frame, "map");
    nh.param<std::string>("robot_frame", robot_frame, "base_link");

    // 创建PathRecorder对象
    PointRecorder point_recorder(file_path);

    // 创建tf监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // input thread
    std::thread input_thread(input_detect, nh);

    // 主循环
    ros::Rate rate(10.0);
    while (nh.ok())
    {
        mutex.lock();
        if (save_point)
        {
            save_point = false;
            mutex.unlock();
            geometry_msgs::TransformStamped transform_stamped;
            try
            {
                transform_stamped = tf_buffer.lookupTransform(global_frame, robot_frame, ros::Time(0));
                point_recorder.add_pose(transform_stamped);
                ROS_INFO("Save point: [%lf, %lf, %lf]",
                         transform_stamped.transform.translation.x,
                         transform_stamped.transform.translation.y,
                         transform_stamped.transform.translation.z);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s", ex.what());
                ros::Duration(1.0).sleep();
                continue;
            }
        }
        mutex.unlock();
        rate.sleep();
    }

    return 0;
}
