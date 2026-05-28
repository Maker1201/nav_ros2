#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <motion_level_control.h>

#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

using namespace std::chrono_literals;

class BpxDriverNode : public rclcpp::Node {
public:
    BpxDriverNode() : Node("bpx_driver_node") {
        // Declare parameters
        declare_parameter("robot_ip", std::string(bpx_sdk::DEFAULT_SERVER_IP));
        declare_parameter("state_upload_rate_hz", 100);
        declare_parameter("motion_command_rate_hz", 50);
        declare_parameter("state_publish_rate_hz", 50);
        declare_parameter("cmd_vel_timeout_sec", 0.5);
        declare_parameter("odom_frame", std::string("odom"));
        declare_parameter("base_frame", std::string("base_link"));
        declare_parameter("imu_frame", std::string("imu_link"));
        declare_parameter("publish_odom_tf", true);

        // Get parameters
        robot_ip_ = get_parameter("robot_ip").as_string();
        int state_rate = get_parameter("state_upload_rate_hz").as_int();
        int cmd_rate = get_parameter("motion_command_rate_hz").as_int();
        int pub_rate = get_parameter("state_publish_rate_hz").as_int();
        cmd_vel_timeout_ = std::chrono::duration<double>(
            get_parameter("cmd_vel_timeout_sec").as_double());
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        imu_frame_ = get_parameter("imu_frame").as_string();
        publish_odom_tf_ = get_parameter("publish_odom_tf").as_bool();

        // Initialize SDK
        motion_.setRobotIp(robot_ip_.c_str());
        motion_.setRobotStateUploadPort(bpx_sdk::DEFAULT_CLIENT_ROBOT_STATE_UDP_PORT);
        motion_.setTcpLocalPort(0);
        motion_.setRobotStateUploadRate(static_cast<uint16_t>(state_rate));
        motion_.setMotionCommandRate(static_cast<uint16_t>(cmd_rate));

        if (!motion_.connect()) {
            RCLCPP_ERROR(get_logger(), "Failed to connect to robot at %s — driver running without robot data", robot_ip_.c_str());
        } else {
            RCLCPP_INFO(get_logger(), "Connected to BPX robot at %s", robot_ip_.c_str());
        }

        // Publishers
        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        battery_pub_ = create_publisher<std_msgs::msg::UInt8>("bpx/battery", 10);
        battery_current_pub_ = create_publisher<std_msgs::msg::Float32>("bpx/battery_current", 10);
        motion_state_pub_ = create_publisher<std_msgs::msg::UInt8>("bpx/motion_state", 10);
        gait_pub_ = create_publisher<std_msgs::msg::UInt8>("bpx/gait", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // Subscriber
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            std::bind(&BpxDriverNode::cmdVelCallback, this, std::placeholders::_1));

        // Services
        stand_up_srv_ = create_service<std_srvs::srv::Trigger>(
            "bpx/stand_up",
            std::bind(&BpxDriverNode::standUpCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        sit_down_srv_ = create_service<std_srvs::srv::Trigger>(
            "bpx/sit_down",
            std::bind(&BpxDriverNode::sitDownCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        damping_srv_ = create_service<std_srvs::srv::Trigger>(
            "bpx/damping",
            std::bind(&BpxDriverNode::dampingCallback, this,
                      std::placeholders::_1, std::placeholders::_2));
        upright_srv_ = create_service<std_srvs::srv::Trigger>(
            "bpx/upright",
            std::bind(&BpxDriverNode::uprightCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // State publish timer
        auto pub_period = std::chrono::milliseconds(1000 / pub_rate);
        state_timer_ = create_wall_timer(
            pub_period, std::bind(&BpxDriverNode::stateTimerCallback, this));

        // Velocity command timer (send at motion command rate)
        auto cmd_period = std::chrono::milliseconds(1000 / cmd_rate);
        vel_timer_ = create_wall_timer(
            cmd_period, std::bind(&BpxDriverNode::velTimerCallback, this));

        RCLCPP_INFO(get_logger(), "BPX driver node started");
    }

    ~BpxDriverNode() override {
        motion_.setVelocityControlFlag(false);
        motion_.disconnect();
    }

private:
    // ---------- Callbacks ----------

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        last_cmd_vel_ = *msg;
        last_cmd_vel_time_ = now();
        velocity_active_ = true;
        motion_.setVelocityControlFlag(true);
    }

    void velTimerCallback() {
        std::lock_guard<std::mutex> lock(vel_mutex_);
        if (!velocity_active_) return;

        // Check timeout
        if ((now() - last_cmd_vel_time_) > cmd_vel_timeout_) {
            // Timeout: send zero velocity and disable velocity control
            motion_.setVelocity(0.0f, 0.0f, 0.0f);
            motion_.setVelocityControlFlag(false);
            velocity_active_ = false;
            RCLCPP_WARN(get_logger(), "cmd_vel timeout, stopping robot");
            return;
        }

        // Robot must be in stand-up state (6) to accept velocity commands
        uint8_t motion_state;
        if (motion_.getCurrentMotionState(&motion_state)) {
            if (motion_state != 6) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
                    "Robot motion state is %d, not 6 (stand up). "
                    "Velocity commands will be ignored. Call /bpx/stand_up service first.",
                    motion_state);
                return;
            }
        }

        float vx = static_cast<float>(last_cmd_vel_.linear.x);
        float vy = static_cast<float>(last_cmd_vel_.linear.y);
        float wz = static_cast<float>(last_cmd_vel_.angular.z);
        motion_.setVelocity(vx, vy, wz);
    }

    void stateTimerCallback() {
        publishJointStates();
        publishImu();
        publishOdometry();
        publishBattery();
        publishMotionState();
    }

    // ---------- Publish helpers ----------

    void publishJointStates() {
        float pos[12], vel[12], tau[12];
        bool has_pos = motion_.getJointPosition(pos);
        bool has_vel = motion_.getJointVelocity(vel);
        bool has_tau = motion_.getJointTorque(tau);

        if (!has_pos) return;

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.header.frame_id = "";

        msg.name = {
            "fl_hip_roll_joint", "fl_hip_pitch_joint", "fl_knee_joint",
            "fr_hip_roll_joint", "fr_hip_pitch_joint", "fr_knee_joint",
            "hl_hip_roll_joint", "hl_hip_pitch_joint", "hl_knee_joint",
            "hr_hip_roll_joint", "hr_hip_pitch_joint", "hr_knee_joint",
        };

        msg.position.assign(pos, pos + 12);
        if (has_vel) msg.velocity.assign(vel, vel + 12);
        if (has_tau) msg.effort.assign(tau, tau + 12);

        joint_state_pub_->publish(msg);
    }

    void publishImu() {
        float quat[4], acc[3], omega[3];
        bool has_quat = motion_.getImuQuat(quat);
        bool has_acc = motion_.getImuAcc(acc);
        bool has_omega = motion_.getImuOmega(omega);

        if (!has_quat && !has_acc && !has_omega) return;

        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = now();
        msg.header.frame_id = imu_frame_;

        if (has_quat) {
            // SDK returns quaternion as (w, x, y, z), ROS uses (x, y, z, w)
            msg.orientation.w = quat[0];
            msg.orientation.x = quat[1];
            msg.orientation.y = quat[2];
            msg.orientation.z = quat[3];
            msg.orientation_covariance = {
                0.01, 0.0,  0.0,
                0.0,  0.01, 0.0,
                0.0,  0.0,  0.01};
        } else {
            msg.orientation_covariance[0] = -1.0;  // mark as invalid
        }

        if (has_omega) {
            msg.angular_velocity.x = omega[0];
            msg.angular_velocity.y = omega[1];
            msg.angular_velocity.z = omega[2];
            msg.angular_velocity_covariance = {
                0.02, 0.0,  0.0,
                0.0,  0.02, 0.0,
                0.0,  0.0,  0.02};
        }

        if (has_acc) {
            msg.linear_acceleration.x = acc[0];
            msg.linear_acceleration.y = acc[1];
            msg.linear_acceleration.z = acc[2];
            msg.linear_acceleration_covariance = {
                0.04, 0.0,  0.0,
                0.0,  0.04, 0.0,
                0.0,  0.0,  0.04};
        }

        imu_pub_->publish(msg);
    }

    void publishOdometry() {
        float vel_body[3], quat[4];
        bool has_vel = motion_.getCurrentVelocityBody(vel_body);
        bool has_quat = motion_.getImuQuat(quat);

        if (!has_vel || !has_quat) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                "Odometry data unavailable: velocity=%d, imu_quat=%d. "
                "Is the robot connected and standing up?",
                has_vel, has_quat);
            return;
        }

        auto current_time = now();
        double dt = 0.0;
        if (last_odom_time_.nanoseconds() > 0) {
            dt = (current_time - last_odom_time_).seconds();
        }
        last_odom_time_ = current_time;

        // Clamp dt to avoid integration jumps on time reset / first call
        if (dt < 0.0 || dt > 1.0) dt = 0.0;

        // Transform body-frame velocity to world frame using IMU quaternion
        // SDK quaternion is (w, x, y, z); v_world = q * v_body * q_conj
        double qw = quat[0], qx = quat[1], qy = quat[2], qz = quat[3];
        double vx_w = (qw * qw + qx * qx - qy * qy - qz * qz) * vel_body[0] +
                      (2.0 * qx * qy - 2.0 * qw * qz) * vel_body[1];
        double vy_w = (2.0 * qx * qy + 2.0 * qw * qz) * vel_body[0] +
                      (qw * qw - qx * qx + qy * qy - qz * qz) * vel_body[1];

        pose_x_ += vx_w * dt;
        pose_y_ += vy_w * dt;

        auto msg = nav_msgs::msg::Odometry();
        msg.header.stamp = current_time;
        msg.header.frame_id = odom_frame_;
        msg.child_frame_id = base_frame_;

        msg.pose.pose.position.x = pose_x_;
        msg.pose.pose.position.y = pose_y_;
        msg.pose.pose.position.z = 0.0;
        // SDK returns quaternion as (w, x, y, z), ROS uses (x, y, z, w)
        msg.pose.pose.orientation.w = qw;
        msg.pose.pose.orientation.x = qx;
        msg.pose.pose.orientation.y = qy;
        msg.pose.pose.orientation.z = qz;

        msg.twist.twist.linear.x = vel_body[0];
        msg.twist.twist.linear.y = vel_body[1];
        msg.twist.twist.angular.z = vel_body[2];

        // Pose covariance (x, y, yaw dominant)
        msg.pose.covariance = {
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e-9,0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e-9,0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e-9,0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.01};
        // Twist covariance
        msg.twist.covariance = {
            0.01, 0.0,  0.0, 0.0, 0.0, 0.0,
            0.0,  0.01, 0.0, 0.0, 0.0, 0.0,
            0.0,  0.0,  1e-9,0.0, 0.0, 0.0,
            0.0,  0.0,  0.0, 1e-9,0.0, 0.0,
            0.0,  0.0,  0.0, 0.0, 1e-9,0.0,
            0.0,  0.0,  0.0, 0.0, 0.0, 0.02};

        odom_pub_->publish(msg);

        if (publish_odom_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = current_time;
            tf.header.frame_id = odom_frame_;
            tf.child_frame_id = base_frame_;
            tf.transform.translation.x = pose_x_;
            tf.transform.translation.y = pose_y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation.w = qw;
            tf.transform.rotation.x = qx;
            tf.transform.rotation.y = qy;
            tf.transform.rotation.z = qz;
            tf_broadcaster_->sendTransform(tf);
        }
    }

    void publishBattery() {
        uint8_t level;
        if (motion_.getBatteryLevel(&level)) {
            auto msg = std_msgs::msg::UInt8();
            msg.data = level;
            battery_pub_->publish(msg);
        }

        float current;
        if (motion_.getBatteryCurrent(&current)) {
            auto msg = std_msgs::msg::Float32();
            msg.data = current;
            battery_current_pub_->publish(msg);
        }
    }

    void publishMotionState() {
        uint8_t state;
        if (motion_.getCurrentMotionState(&state)) {
            auto msg = std_msgs::msg::UInt8();
            msg.data = state;
            motion_state_pub_->publish(msg);
        }

        uint8_t gait;
        if (motion_.getCurrentGait(&gait)) {
            auto msg = std_msgs::msg::UInt8();
            msg.data = gait;
            gait_pub_->publish(msg);
        }
    }

    // ---------- Service callbacks ----------

    void standUpCallback(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (motion_.setStandUp()) {
            pose_x_ = 0.0;
            pose_y_ = 0.0;
            res->success = true;
            res->message = "Stand up command sent";
        } else {
            res->success = false;
            res->message = "Failed to send stand up command";
        }
    }

    void sitDownCallback(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
        {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            motion_.setVelocityControlFlag(false);
            velocity_active_ = false;
        }
        if (motion_.setSitDown()) {
            res->success = true;
            res->message = "Sit down command sent";
        } else {
            res->success = false;
            res->message = "Failed to send sit down command";
        }
    }

    void dampingCallback(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
        {
            std::lock_guard<std::mutex> lock(vel_mutex_);
            motion_.setVelocityControlFlag(false);
            velocity_active_ = false;
        }
        if (motion_.setDamping()) {
            res->success = true;
            res->message = "Damping command sent";
        } else {
            res->success = false;
            res->message = "Failed to send damping command";
        }
    }

    void uprightCallback(const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                         std_srvs::srv::Trigger::Response::SharedPtr res) {
        if (motion_.setUpright()) {
            pose_x_ = 0.0;
            pose_y_ = 0.0;
            res->success = true;
            res->message = "Upright command sent";
        } else {
            res->success = false;
            res->message = "Failed to send upright command";
        }
    }

    // ---------- Members ----------
    bpx_sdk::MotionLevelControl motion_;

    std::string robot_ip_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string imu_frame_;
    bool publish_odom_tf_ = true;

    // Odometry integration
    double pose_x_ = 0.0;
    double pose_y_ = 0.0;
    rclcpp::Time last_odom_time_;

    // Velocity control
    std::mutex vel_mutex_;
    geometry_msgs::msg::Twist last_cmd_vel_;
    rclcpp::Time last_cmd_vel_time_;
    bool velocity_active_ = false;
    std::chrono::duration<double> cmd_vel_timeout_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr battery_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_current_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr motion_state_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr gait_pub_;

    // Subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stand_up_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr sit_down_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr damping_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr upright_srv_;

    // Timers
    rclcpp::TimerBase::SharedPtr state_timer_;
    rclcpp::TimerBase::SharedPtr vel_timer_;

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BpxDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
