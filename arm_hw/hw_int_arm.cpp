#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>

class RoboticArmHWInterface : public hardware_interface::RobotHW {
public:
    RoboticArmHWInterface() {
        // Initialize joint names
        joint_names_ = {"Joint_1", "Joint_2", "Joint_3", "Joint_4","Joint_5","Joint_6"}; // joint_names_ = {"iiwa_joint_1","iiwa_joint_2","iiwa_joint_3","iiwa_joint_4","iiwa_joint_5","iiwa_joint_6","iiwa_joint_7"};
        num_joints_ = joint_names_.size();

        // Initialize joint states and commands
        joint_position_.resize(num_joints_, 0.0);
        joint_velocity_.resize(num_joints_, 0.0);
        joint_effort_.resize(num_joints_, 0.0);
        joint_position_command_.resize(num_joints_, 0.0);

        // Register joint interfaces
        for (size_t i = 0; i < num_joints_; ++i) {
            // Joint state interface
            hardware_interface::JointStateHandle state_handle(
                joint_names_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
            joint_state_interface_.registerHandle(state_handle);

            // Position joint interface
            hardware_interface::JointHandle position_handle(
                joint_state_interface_.getHandle(joint_names_[i]), &joint_position_command_[i]);
            position_joint_interface_.registerHandle(position_handle);
        }
        end_effector_names_ = {"Finger_1","Finger_2"};
        num_fings = end_effector_names_.size();

        end_effector_position_.resize(num_fings, 0.0);
        end_effector_velocity_.resize(num_fings, 0.0);
        end_effector_effort_.resize(num_fings, 0.0);
        end_effector_command_.resize(num_fings, 0.0);

        for (size_t i = 0; i < num_fings; ++i) {
            end_effector_position_[i] = 0.0;
            end_effector_velocity_[i] = 0.0;
            end_effector_effort_[i] = 0.0;
            end_effector_command_[i] = 0.0;

            // Register joint state interface for end effector joints
            hardware_interface::JointStateHandle state_handle(
                end_effector_names_[i],
                &end_effector_position_[i],
                &end_effector_velocity_[i],
                &end_effector_effort_[i]
            );
            joint_state_interface_.registerHandle(state_handle);

            // Register position command interface for end effector joints
            hardware_interface::JointHandle pos_handle(
                joint_state_interface_.getHandle(end_effector_names_[i]),
                &end_effector_command_[i]
            );
            position_joint_interface_.registerHandle(pos_handle);
        }

        // Register interfaces
        registerInterface(&joint_state_interface_);
        registerInterface(&position_joint_interface_);
    }

    void read() {

    }

    void write() {

    }

private:
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface position_joint_interface_;

    std::vector<std::string> joint_names_;
    std::vector<std::string> end_effector_names_;
    size_t num_joints_;
    size_t num_fings;
    std::vector<double> joint_position_;
    std::vector<double> joint_velocity_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_position_command_;
    
    std::vector<double> end_effector_position_;
    std::vector<double> end_effector_velocity_;
    std::vector<double> end_effector_effort_;
    std::vector<double> end_effector_command_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_hw");
    ros::NodeHandle nh;

    RoboticArmHWInterface robotic_arm_hw;
    controller_manager::ControllerManager cm(&robotic_arm_hw, nh);

    ros::Rate rate(50);  // 50 Hz control loop
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        //robotic_arm_hw.read();
        cm.update(ros::Time::now(), ros::Duration(0.02));
        //robotic_arm_hw.write();
        rate.sleep();
    }

    return 0;
}
