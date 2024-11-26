#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "ik_service/PoseIK.h"

int main(int argc, char *argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "pose_ik_client");
    ros::NodeHandle n;

    // 创建服务客户端
    ros::ServiceClient client = n.serviceClient<ik_service::PoseIK>("pose_ik");

    // 等待服务可用
    if (!ros::service::waitForService("pose_ik", ros::Duration(5.0))) {
        ROS_ERROR("Service pose_ik is not available.");
        return 1;
    }

    // 设置目标位姿
    geometry_msgs::Pose part_pose;
    part_pose.position.x = 0.5;
    part_pose.position.y = 0.0;
    part_pose.position.z = 0.0;
    part_pose.orientation.x = 0.0;
    part_pose.orientation.y = 0.0;
    part_pose.orientation.z = 0.0;
    part_pose.orientation.w = 1.0;

    // 创建服务请求
    ik_service::PoseIK pose_ik;
    pose_ik.request.part_pose = part_pose;

    // 调用服务并处理响应
    if (client.call(pose_ik)) {
        ROS_INFO("Call to ik_service returned [%d] solutions", static_cast<int>(pose_ik.response.num_sols));

        for (int i = 0; i < pose_ik.response.num_sols; ++i) {
            const auto& joint_angles = pose_ik.response.joint_solutions[i].joint_angles;
            ROS_INFO("Solution %d: [%f, %f, %f, %f, %f, %f]", i,
                     joint_angles[0], joint_angles[1], joint_angles[2],
                     joint_angles[3], joint_angles[4], joint_angles[5]);
        }
    } else {
        ROS_ERROR("Failed to call service pose_ik");
    }

    return 0;
}

