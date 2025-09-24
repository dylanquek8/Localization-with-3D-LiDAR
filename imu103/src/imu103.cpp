#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class ImuRep103Node : public rclcpp::Node
{
public:
    ImuRep103Node() : Node("imu103")
    {
        // Parameters
        this->declare_parameter<std::string>("input_topic", "raw_imu");
        this->declare_parameter<std::vector<double>>("calib_quaternion",
            std::vector<double>{-0.701591, 0.712543, -0.00591461, -0.00427578});
        this->declare_parameter<std::vector<double>>("calib_translation",
            std::vector<double>{0.00425, 0.00418, -0.00446});

        std::string input_topic;
        std::vector<double> q_vec, t_vec;
        this->get_parameter("input_topic", input_topic);
        this->get_parameter("calib_quaternion", q_vec);
        this->get_parameter("calib_translation", t_vec);

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic, 10,
            std::bind(&ImuRep103Node::imuCallback, this, std::placeholders::_1));

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu103", 10);

        // Calibration quaternion (rotation from IMU frame -> LiDAR/base frame)
        calib_q_.setValue(q_vec[0], q_vec[1], q_vec[2], q_vec[3]);
        calib_q_.normalize();

        // Translation is unused for orientation/vel/acc but may be used if needed
        calib_t_[0] = t_vec[0];
        calib_t_[1] = t_vec[1];
        calib_t_[2] = t_vec[2];

        RCLCPP_INFO(this->get_logger(), "IMU REP-103 conversion node started.");
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        sensor_msgs::msg::Imu imu_out;
        imu_out.header = msg->header; // keep same timestamp/frame_id

        // --- Convert units ---
        // Linear acceleration: g -> m/s^2
        tf2::Vector3 acc(msg->linear_acceleration.x * 9.80665,
                         msg->linear_acceleration.y * 9.80665,
                         msg->linear_acceleration.z * 9.80665);

        // Angular velocity: deg/s -> rad/s
        tf2::Vector3 gyro(msg->angular_velocity.x * M_PI / 180.0,
                          msg->angular_velocity.y * M_PI / 180.0,
                          msg->angular_velocity.z * M_PI / 180.0);

        // --- Apply rotation calibration ---
        tf2::Matrix3x3 R(calib_q_);
        acc = R * acc;
        gyro = R * gyro;

        // --- Fill converted IMU message ---
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();

        imu_out.angular_velocity.x = gyro.x();
        imu_out.angular_velocity.y = gyro.y();
        imu_out.angular_velocity.z = gyro.z();

        // Set angular velocity covariance (gyro)
        imu_out.angular_velocity_covariance[0] = 3.233978e-10; // var(wx)
        imu_out.angular_velocity_covariance[4] = 2.146677e-10; // var(wy)
        imu_out.angular_velocity_covariance[8] = 2.522456e-10; // var(wz)

        // Set linear acceleration covariance (accel)
        imu_out.linear_acceleration_covariance[0] = 1.691062e-04; // var(ax)
        imu_out.linear_acceleration_covariance[4] = 1.959819e-04; // var(ay)
        imu_out.linear_acceleration_covariance[8] = 9.313394e-05; // var(az)


        // For orientation: apply extrinsic rotation to raw orientation if provided
        if (!(msg->orientation.x == 0 &&
              msg->orientation.y == 0 &&
              msg->orientation.z == 0 &&
              msg->orientation.w == 0))
        {
            tf2::Quaternion q_in, q_out;
            tf2::fromMsg(msg->orientation, q_in);

            // Convert IMU orientation into calibrated frame
            q_out = calib_q_ * q_in;
            q_out.normalize();
            imu_out.orientation = tf2::toMsg(q_out);
        }
        else
        {
            imu_out.orientation.x = 0.0;
            imu_out.orientation.y = 0.0;
            imu_out.orientation.z = 0.0;
            imu_out.orientation.w = 1.0;
        }

        // Pass through covariance if available
        //imu_out.orientation_covariance = msg->orientation_covariance;
        //imu_out.angular_velocity_covariance = msg->angular_velocity_covariance;
        //imu_out.linear_acceleration_covariance = msg->linear_acceleration_covariance;

        imu_pub_->publish(imu_out);
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    tf2::Quaternion calib_q_;
    double calib_t_[3];
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuRep103Node>());
    rclcpp::shutdown();
    return 0;
}
