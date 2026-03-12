#include "quad_rover_kinematics/rover.h"

Rover::Rover(const rclcpp::NodeOptions &options, const std::string &config_file_path)
    : Node("quad_rover_kinematics", options), 
        
        sim_time_(0, 0, RCL_SYSTEM_TIME),
        publish_period_(rclcpp::Duration::from_seconds(0.05)),
        kinematic_(),  // Default-initialized kinematic_
        L_(1), W_(1), wheel_diameter_(0.25), poles_pairs_number_(15),
        pose_covariance_diagonal_({0.001, 0.001, 0.001, 0.001, 0.001, 0.001}),
        twist_covariance_diagonal_({0.001, 0.001, 0.001, 0.001, 0.001, 0.001}),
        use_sim_time_(false),
        publish_rate_(10.0),
        base_frame_id_("/base_footprint"),
        odom_frame_id_("/odom"),
        enable_odom_tf_(false),
        control_mode_(USER_CONTROL),
        communication_state_(CONNECTION_DISCONNECTED)

{
    readConfig(config_file_path);
    setCmdVelocity(0, 0, 0);

    // Reassign kinematic_ using updated W_, L_, etc.
    kinematic_ = Kinematics(W_, L_, pose_covariance_diagonal_, twist_covariance_diagonal_);
    kinematic_.setFrames(odom_frame_id_, base_frame_id_);
    kinematic_.setSteeringRadius(max_steering_radius_, min_steering_radius_);
    kinematic_.setWheelRadius(wheel_diameter_ / 2.0);
    kinematic_.setMinERPM(min_erpm_);
    kinematic_.setPolesPairsNumber(poles_pairs_number_);
    kinematic_.setMotorGearRatio(motor_gear_ratio_);

    timer_ = this->create_wall_timer(
        publish_period_.to_chrono<std::chrono::milliseconds>(),  // Use publish_period_
        std::bind(&Rover::onUpdate, this));

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

    cmd_vel_sub_ = this->create_subscription<rex_interfaces::msg::RoverControl>(
        "/MQTT/RoverControl", qos, 
        [this](const rex_interfaces::msg::RoverControl::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received RoverControl message");
            this->cmdVelCallback(msg);
        });
    cmd_vel_sub_auto_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", qos, 
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received Twist message");
            this->cmdVelAutoCallback(msg);
        });
    status_sub_ = this->create_subscription<rex_interfaces::msg::RoverStatus>(
        "/MQTT/RoverStatus", qos, 
        [this](const rex_interfaces::msg::RoverStatus::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received RoverStatus message");
            this->statusCallback(msg);
        });
    wheels_vel_feedback_sub_ = this->create_subscription<rex_interfaces::msg::VescStatus>(
        "/CAN/RX/vesc_status",
        rclcpp::QoS(1000).reliable(), // changed from rclcpp::SensorDataQoS()
        [this](const rex_interfaces::msg::VescStatus::SharedPtr msg) {
            RCLCPP_DEBUG(this->get_logger(), "Received VescStatus message");
            this->feedbackCallback(msg);
        });

    wheels_vel_pub_ = this->create_publisher<rex_interfaces::msg::Wheels>("/CAN/TX/set_motor_vel", qos);

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/kinematic/odometry", qos);
    tf_odom_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);

    reset_odom_srv_ = this->create_service<std_srvs::srv::Empty>(
        "reset_odometry",
        std::bind(&Rover::resetOdometryCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Subscribe to the /clock topic
    clock_subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock",
        10,
        std::bind(&Rover::clockCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Simulation Time Node started.");
}

void Rover::readConfig(const std::string &config_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(config_file_path);
        L_ = config["wheels_distance_length"].as<double>();
        W_ = config["wheels_distance_width"].as<double>();
        wheel_diameter_ = config["wheel_diameter"].as<double>();
        poles_pairs_number_ = config["poles_pair_number"].as<int>();
        motor_gear_ratio_ = config["motor_gear_ratio"].as<double>();
        min_erpm_ = config["min_erpm"].as<double>();
        max_steering_radius_ = config["max_steering_radius"].as<double>();
        min_steering_radius_ = config["min_steering_radius"].as<double>();
        pose_covariance_diagonal_ = config["pose_covariance_diagonal"].as<std::vector<double>>();
        twist_covariance_diagonal_ = config["twist_covariance_diagonal"].as<std::vector<double>>();
        publish_rate_ = config["publish_rate"].as<double>();
        use_sim_time_ = config["use_sim_time"].as<bool>();
        base_frame_id_ = config["base_frame_id"].as<std::string>();
        odom_frame_id_ = config["odom_frame_id"].as<std::string>();
        enable_odom_tf_ = config["enable_odom_tf"].as<bool>();

        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

        RCLCPP_INFO(this->get_logger(), "YAML configuration loaded successfully!");

    } catch (const YAML::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading configuration file: %s", e.what());
    }
}

void Rover::resetOdometryCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Empty::Response> /*response*/)
{
    RCLCPP_INFO(this->get_logger(), "Resetting odometry");
    // Reset relevant odometry data here
    kinematic_.resetOdometry();
}

void Rover::setCmdVelocity(double x_vel, double y_vel, double speed) {
    rover_cmd_velocity_.header.stamp = use_sim_time_ ? sim_time_ : this->get_clock()->now();
    rover_cmd_velocity_.x_axis = x_vel;
    rover_cmd_velocity_.y_axis = y_vel;
    rover_cmd_velocity_.vel = speed;

    RCLCPP_DEBUG(this->get_logger(), "CMD velocity reset!");
}

void Rover::onUpdate()
{
    rclcpp::Time time_curr = use_sim_time_ ? sim_time_ : this->get_clock()->now();
    //kinematic_.setSimTime(sim_time_);

    static uint8_t counter = 0;

    // 1. Update odometry
    auto feedback_msg = *(rover_wheels_velocity_feedback_buffer_.readFromNonRT());
    kinematic_.updateOdometry(time_curr, feedback_msg);
    odom_pub_->publish(kinematic_.getOdom());

    if(enable_odom_tf_) {
        tf_odom_pub_->publish(kinematic_.getTF());
    }

    // 2. Fetch current command velocity
    auto cmd_vel = *(rover_cmd_velocity_buffer_.readFromNonRT());

    // 3. Check connection state
    if (communication_state_ != CONNECTION_CONNECTED) {
        rover_wheels_velocity_ = kinematic_.brake(time_curr);
        cmd_vel.vel = 0.0;
        cmd_vel.x_axis = 0.0;
        cmd_vel.y_axis = 0.0;
        rover_cmd_velocity_buffer_.writeFromNonRT(cmd_vel);

        if (counter % 100 == 0) {
            RCLCPP_WARN(this->get_logger(), "'/MQTT/RoverStatus.ConnectionStatus' is not 'Opened'!");
        }
        counter++;
    }
    else {
        // 4. Control logic
        switch (control_mode_) {
            case USER_CONTROL:
                if      (cmd_vel.mode == ADVANCE_MODE) rover_wheels_velocity_ = kinematic_.advanceRPMKinematics(time_curr, cmd_vel.x_axis, cmd_vel.vel, feedback_msg);
                else if (cmd_vel.mode == CRAB_MODE)    rover_wheels_velocity_ = kinematic_.crabDriveKinematics(time_curr, cmd_vel.x_axis, cmd_vel.y_axis, cmd_vel.vel, feedback_msg);
                else if (cmd_vel.mode == SPIN_MODE)    rover_wheels_velocity_ = kinematic_.spinDriveKinematics(time_curr, cmd_vel.vel, feedback_msg);
                else                                   rover_wheels_velocity_ = kinematic_.brake(time_curr);
                break;
            case AUTONOMY_CONTROL:
                //rover_wheels_velocity_ = kinematic_.advanceRPMKinematics(time_curr, cmd_vel.x_axis, cmd_vel.vel, feedback_msg);
                if      (cmd_vel.mode == ADVANCE_MODE) rover_wheels_velocity_ = kinematic_.advanceRPMKinematics(time_curr, cmd_vel.x_axis, cmd_vel.vel , feedback_msg);
                else if (cmd_vel.mode == SPIN_MODE)    rover_wheels_velocity_ = kinematic_.spinDriveKinematics(time_curr, cmd_vel.vel, feedback_msg);
                else                                   rover_wheels_velocity_ = kinematic_.brake(time_curr);
                break;
            default:
                rover_wheels_velocity_ = kinematic_.brake(time_curr);
                break;
        }
    }

    // 5. Publish final wheel velocities
    wheels_vel_pub_->publish(rover_wheels_velocity_);
}

void Rover::cmdVelCallback(const rex_interfaces::msg::RoverControl::SharedPtr msg) {
    if (control_mode_ == USER_CONTROL) {
        rover_cmd_velocity_buffer_.writeFromNonRT(*msg);
        RCLCPP_DEBUG(this->get_logger(), "Received Cmd velocity msg!");
    }
}

void Rover::feedbackCallback(const rex_interfaces::msg::VescStatus::SharedPtr msg) {

    rover_wheels_velocity_feedback_.header.stamp = use_sim_time_ ? sim_time_ : this->get_clock()->now();

    switch (msg->vesc_id) {
        case 0x50:
            this->rover_wheels_velocity_feedback_.front_left.drive.set_value = msg->erpm * M_PI / 30 * wheel_diameter_ / 2 / poles_pairs_number_;
            break;
        case 0x51:
            this->rover_wheels_velocity_feedback_.front_right.drive.set_value = msg->erpm * M_PI / 30 * wheel_diameter_ / 2 / poles_pairs_number_;
            break;
        case 0x52:
            this->rover_wheels_velocity_feedback_.rear_left.drive.set_value =  msg->erpm * M_PI / 30 * wheel_diameter_ / 2 / poles_pairs_number_;
            break;
        case 0x53:
            this->rover_wheels_velocity_feedback_.rear_right.drive.set_value =  msg->erpm * M_PI / 30 * wheel_diameter_ / 2 / poles_pairs_number_;
            break;
        case 0x60:
            this->rover_wheels_velocity_feedback_.front_left.turn.set_value = msg->precise_pos;
            break;
        case 0x61:
            this->rover_wheels_velocity_feedback_.front_right.turn.set_value = msg->precise_pos; //(-1)*
            break;
        case 0x62:
            this->rover_wheels_velocity_feedback_.rear_right.turn.set_value = msg->precise_pos; //(-1)*
            break;
        case 0x63:
            this->rover_wheels_velocity_feedback_.rear_left.turn.set_value = msg->precise_pos;
            break;
        default:
            break;
    }

    rover_wheels_velocity_feedback_buffer_.writeFromNonRT(rover_wheels_velocity_feedback_);
}    

void Rover::statusCallback(const rex_interfaces::msg::RoverStatus::SharedPtr msg) {
    RCLCPP_DEBUG(this->get_logger(), "Received RoverStatus message");
    communication_state_ = msg->communication_state;
    control_mode_ = msg->control_mode;
}

void Rover::cmdVelAutoCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (control_mode_ == AUTONOMY_CONTROL) {

        double sign_x = (msg->linear.x > 0.0) ? 1.0 : -1.0;

        if(msg->linear.x != 0.0 && msg->angular.z != 0.0) {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = msg->linear.x;
            rover_cmd_velocity_.x_axis = sign_x * msg->angular.z;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else if(msg->linear.x != 0.0 && msg->angular.z == 0.0) {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = msg->linear.x;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else if(msg->linear.x == 0.0 && msg->angular.z != 0.0) {
            rover_cmd_velocity_.mode = SPIN_MODE;
            rover_cmd_velocity_.vel = msg->angular.z;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }
        else {
            rover_cmd_velocity_.mode = ADVANCE_MODE;
            rover_cmd_velocity_.vel = 0.0;
            rover_cmd_velocity_.x_axis = 0.0;
            rover_cmd_velocity_buffer_.writeFromNonRT(rover_cmd_velocity_);
        }


        RCLCPP_INFO(this->get_logger(), "Received Cmd velocity msg from autonomy!");
    }
}

void Rover::clockCallback(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
    sim_time_ = msg->clock;
    // ... handle clock messages ...
}
