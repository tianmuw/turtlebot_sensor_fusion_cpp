#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "irobot_create_msgs/msg/mouse.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "marvelmind_ros2_msgs/msg/hedgepositionaddressed.hpp"
#include <queue>
#include <random>
#include <Eigen/Dense>
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"

/*
This program is to estimate the linear velocity and traveled distance on the turtlebot4, 
currently we have the imu sensor, the optical mouse sensor on the bottom of the robot,
and the ultrasonic marvelmind coordinate systems. Use some knowledge of sensor fusion to
estimate the true states.
*/

/*
Define a struct called measurement to store
the measurement information.
In this program, we are focusing on 2D problems, 
so we only need data in x and y axis.
Also we need the time interval dt,
and the measurement source we get, which is a string.
*/
struct measurement
{
    std_msgs::msg::Float64 time;
    std_msgs::msg::Float64 mea_x;
    std_msgs::msg::Float64 mea_y;
    float mea_vx;
    float mea_vy;
};

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class SensorFusionNode : public rclcpp::Node
{
private:
    //The namespace our lab uses.
    std::string robotNamespace;

    /*
    accInput: a vector, containning the acceleration data in x and y axis.
    state: a vector, containning [x, y, vx, vy].
    P: covariance matrix.
    */
    Eigen::Vector2f accInput  {{0.0, 0.0}};
    Eigen::Vector4f state  {{0.0, 0.0, 0.0, 0.0}};
    Eigen::Matrix4f P = Eigen::Matrix4f::Random(4, 4);
    // Make sure the matrix is positive definite
    P = P * P.transpose();

    Eigen::Matrix4f Q = Eigen::Matrix4f::Random(4, 4);
    Eigen::Matrix4f R = Eigen::Matrix2f::Random(2, 2);

    /*
    w: The Gaussian noise with mean = 0.0 and std = 0.8.
    */
    Eigen::Vector4f w = 0.8 * Eigen::Vector4f::Random(4, 1) + 0.0;

    //Create a queue to store the measurement information.
    std::queue<measurement> meas_queue;

    //Topic name
    std::string imuTopic;
    std::string mouseTopic;
    std::string marvelTopic;

    //Store the previous time, which is used to calculate the dt.
    float lastTime;

    //Subscribe IMU
    //rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Imu>> subImu;
    //Subscribe Mouse
    //rclcpp::Subscription<irobot_create_msgs::msg::Mouse>::SharedPtr subMouse;
    std::shared_ptr<message_filters::Subscriber<irobot_create_msgs::msg::Mouse>> subMouse;
    //Subscribe Marvelmind
    //rclcpp::Subscription<marvelmind_ros2_msgs::msg::HedgePositionAddressed>::SharedPtr subMarvel;
    std::shared_ptr<message_filters::Subscriber<marvelmind_ros2_msgs::msg::HedgePositionAddressed>> subMarvel;
    std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::Imu, irobot_create_msgs::msg::Mouse,
                                                      marvelmind_ros2_msgs::msg::HedgePositionAddressed>> ts_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubState;

    rclcpp::TimerBase::SharedPtr ownTimer;

    rclcpp::QoS qosProfileSensorData;

    /*
    Imu callback function.
    Get the imu linear acceleration data, then extract the data 
    in x and y axis, store it to our input data.
    */
    
    /*
    Mouse callback function.
    Get the measurement data by the mouse, used to 
    calculate the velocity measurement.
    */
    
    /*
    Marvelmind callback function.
    Get the measurement data by the marvelmind, 
    used to calculate the distance measurement.
    */
    void allCallback(const sensor_msgs::msg::Imu::SharedPtr imuMsg,
                     const irobot_create_msgs::msg::Mouse::SharedPtr mouseMsg,
                     const marvelmind_ros2_msgs::msg::HedgePositionAddressed marvelMsg)
    {
        float currentTime = imuMsg->header.stamp.sec();
        float dt = currentTime - lastTime;
        accInput(0) = imuMsg->linear_acceleration.x;
        accInput(1) = imuMsg->linear_acceleration.y;
        
        measurement meaInfo;
        meaInfo.time = dt;
        meaInfo.mea_x = marvelMsg->x_m;
        meaInfo.mea_y = marvelMsg->y_m;
        meaInfo.mea_vx = mouseMsg->integrated_x / dt;
        meaInfo.mea_vy = mouseMsg->integrated_y / dt;

        meas_queue.push(meaInfo);
    }

    /*
    The program is running in this time callback function.
    */
    void timerCallback()
    {
        measurement meas = meas_queue.pop();
        Eigen::Matrix4f A = {
                {1, 0, meas.time, 0},
                {0, 1, 0, meas.time},
                {0, 0, 1, 0},
                {0, 0, 0, 1}
            };
        Eigen::Matrix<float, 4, 2> B = {
                {0.5 * (meas.time ^ 2), 0},
                {0, 0.5 * (meas.time ^ 2)},
                {meas.time, 0},
                {0, meas.time}
            };
        Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
        Eigen::Vector4f meaE {{meas.mea_x, meas.mea_y, meas.mea_vx, meas.mea_vy}}
        state = kalmanFilter(A, B, H, state, accInput, meaE, P, Q, R, w);
        pubOdomFunc(state);
    }
   
public:
    SensorFusionNode(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "Generate this sensor fusion node %s.", name.c_str());

        //Some basic parameters
        robotNamespace = this->get_parameter("robotNamespace").as_string();
        state = this->get_parameter("robotInitState");

        /*
        Generate the topic string name that we need.
        In this program, we need imu, mouse, and marvelmind topics.
        */
        imuTopic = robotNamespace + "/imu";
        mouseTopic = robotNamespace + "/mouse";
        marvelTopic = robotNamespace + "/hedge_position_addressed";

        //pose_state = this->get_parameter("init_pose");

        lastTime = this->get_clock()->now().nanoseconds * 1e-9;

        /*
        Create the subscription and publisher.
        */
        qosProfileSensorData = rclcpp::QoS(rclcpp::SensorDataQoS());

        //subImu = this->create_subscription<sensor_msgs::msg::Imu>(imuTopic, 10, std::bind(&SensorFusionNode::imuCallback, this, _1));
        subImu = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Imu>>(this, imuTopic, qosProfileSensorData);
        
        //subMouse = this->create_subscription<irobot_create_msgs::msg::Mouse>(mouseTopic, 10, std::bind(&SensorFusionNode::mouseCallback, this, _1));
        subMouse = std::make_shared<message_filters::Subscriber<irobot_create_msgs::msg::Mouse>>(this, mouseTopic, qosProfileSensorData);

        //subMarvel = this->create_subscription<marvelmind_ros2_msgs::msg::HedgePositionAddressed>(marvelTopic, 10, std::bind(&SensorFusionNode::mouseCallback, this, _1));
        subMarvel = std::make_shared<message_filters::Subscriber<marvelmind_ros2_msgs::msg::HedgePositionAddressed>>(this, marvelTopic, qosProfileSensorData);

        ts_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::Imu, geometry_msgs::msg::Twist>>(*subImu, *subMouse, *subMarvel, 10);
        ts_->registerCallback(std::bind(&SensorFusionNode::allCallback, this, _1, _2, _3));



        pubState = this->create_publisher<nav_msgs::msg::Odometry>("/filtered", 10);

        ownTimer = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MyNode::timerCallback, this));

    }


    /*
    The kalman filter process.
    */
    Eigen::Vector4f kalmanFilter(Eigen::Matrix4f A, Eigen::Matrix4f B, Eigen::MatrixXf H, Eigen::Vector4f state, Eigen::Vector2f input, Eigen::Vector2f measurement, 
                                Eigen::Matrix4f P, Eigen::Matrix4f Q, Eigen::Matrix2f R, Eigen::Vector4f w)
    {
        //[x, y, vx, vy]
        Eigen::Vector4f statePred;
        Eigen::Matrix4f PPred;
        Eigen::Vector2f y;
        Eigen::Matrix2f S;
        Eigen::MatrixXf K;
        Eigen::Vector4f stateUpdate;
        Eigen::Matrix4f PUpdate;

        statePred = A * state + B * input + w;
        PPred = A * P * A.transpose() + Q;

        y = measurement - H * statePred;
        S = H * PPred * H.transpose() + R;
        K = PPred * H.transpose() * S.inverse();
        stateUpdate = statePred + K * y;
        PUpdate = (Eigen::Matrix4f::Identity() - K * H) * PPred;

        P = PUpdate;

        return stateUpdate
    }
    
    /*
    Used to publish the information
    */
    void pubStateFunc(Eigen::Vector4f state)
    {
        nav_msgs::msg::Odometry odom;
        float currentTime = this->get_clock()->now();

        odom.header.stamp = currentTime;
        odom.header.frame_id = "odom";

        odom.pose.pose.position.x = state(0);
        odom.pose.pose.position.y = state(1);
        odom.twist.twist.linear.x = state(2);
        odom.twist.twist.linear.y = state(3);

        pubState->publish(odom);
    }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  //Declare the init state [x, y, vx, vy]
  Eigen::Vector4f initState {{0.0, 0.0, 0.0, 0.0}};
  auto node = std::make_shared<SensorFusionNode>();
  node->declare_parameter<std::string>("robotNamespace", "yosemite");
  node->declare_parameter<Eigen::Vector4f>("robotInitState", initState);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}