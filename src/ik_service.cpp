#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <ik_service/PoseIK.h>
#include <ur_kinematics/ur_kin.h>

// 服务回调函数
bool poseIK(ik_service::PoseIK::Request &req, ik_service::PoseIK::Response &res) {
    ROS_INFO("Received a request to compute IK for pose:");
    ROS_INFO_STREAM("Position: [" << req.part_pose.position.x << ", "
                                  << req.part_pose.position.y << ", "
                                  << req.part_pose.position.z << "]");
    ROS_INFO_STREAM("Orientation: [" << req.part_pose.orientation.x << ", "
                                     << req.part_pose.orientation.y << ", "
                                     << req.part_pose.orientation.z << ", "
                                     << req.part_pose.orientation.w << "]");

    // 初始化转换矩阵 (4x4)
    double T[4][4] = {
        {0.0, -1.0,  0.0, req.part_pose.position.x},
        {0.0,  0.0,  1.0, req.part_pose.position.y},
        {-1.0, 0.0,  0.0, req.part_pose.position.z},
        {0.0,  0.0,  0.0, 1.0}
    };

    // 存储逆运动学解
    double q_sols[8][6]; // 最多8个解，每个解6个关节角
    int num_sols = ur_kinematics::inverse(&T[0][0], &q_sols[0][0], 0.0);

    ROS_INFO("Number of IK solutions generated: %d", num_sols);

    // 如果没有解，返回失败
    if (num_sols <= 0) {
        ROS_WARN("No valid IK solutions found.");
        res.num_sols = 0;
        return true; // 返回true表示服务成功响应
    }

    // 将解填充到响应中
    res.num_sols = num_sols;
    res.joint_solutions.resize(num_sols); // 调整vector大小
    for (int i = 0; i < num_sols; ++i) {
        for (int j = 0; j < 6; ++j) {
            res.joint_solutions[i].joint_angles[j] = q_sols[i][j];
        }
    }

    ROS_INFO("IK solutions successfully computed and sent back.");
    return true;
}

int main(int argc, char **argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "ik_service");
    ros::NodeHandle nh;

    // 创建服务
    ros::ServiceServer service = nh.advertiseService("pose_ik", poseIK);
    ROS_INFO("IK Service is ready to receive requests.");

    // 循环等待服务请求
    ros::spin();

    return 0;
}

