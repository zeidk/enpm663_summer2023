#include <string>
#include "qos_subscriber.hpp"
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

double QoSSubscriber::rmw_time_to_seconds(const rmw_time_t &time)
{
    double result = static_cast<double>(time.sec);
    result += 1e-9 * time.nsec;
    return result;
}

void QoSSubscriber::print_qos(const rclcpp::QoS &qos)
{
    const auto &rmw_qos = qos.get_rmw_qos_profile();
    std::cout << "HISTORY POLICY: ";
    switch (rmw_qos.history)
    {
    case RMW_QOS_POLICY_HISTORY_KEEP_LAST:
        std::cout << "keep last";
        break;
    case RMW_QOS_POLICY_HISTORY_KEEP_ALL:
        std::cout << "keep all";
        break;
    default:
        std::cout << "invalid";
    }
    std::cout << " (depth: " << rmw_qos.depth << ')' << std::endl;

    std::cout << "RELIABILITY POLICY: ";
    switch (rmw_qos.reliability)
    {
    case RMW_QOS_POLICY_RELIABILITY_RELIABLE:
        std::cout << "reliable";
        break;
    case RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT:
        std::cout << "best effort";
        break;
    default:
        std::cout << "invalid";
    }
    std::cout << std::endl;

    std::cout << "DURABILITY POLICY: ";
    switch (rmw_qos.durability)
    {
    case RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL:
        std::cout << "transient local";
        break;
    case RMW_QOS_POLICY_DURABILITY_VOLATILE:
        std::cout << "volatile";
        break;
    default:
        std::cout << "invalid";
    }
    std::cout << std::endl;

    std::cout << "DEADLINE: " << rmw_time_to_seconds(rmw_qos.deadline) << std::endl;

    std::cout << "LIFESPAN: " << rmw_time_to_seconds(rmw_qos.lifespan) << std::endl;

    std::cout << "LIVELINESS POLICY: ";
    switch (rmw_qos.liveliness)
    {
    case RMW_QOS_POLICY_LIVELINESS_AUTOMATIC:
        std::cout << "automatic";
        break;
    case RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC:
        std::cout << "manual by topic";
        break;
    default:
        std::cout << "invalid";
    }
    std::cout << " (lease duration: " << rmw_time_to_seconds(rmw_qos.liveliness_lease_duration) << ')' << std::endl;
}

void QoSSubscriber::initialize_()
{
    // initialize the count
    count_ = 0;
    // initialize the message
    msg_ = std_msgs::msg::String();
    // start counter timer
    counter_timer_ = this->create_wall_timer(
        std::chrono::milliseconds((int)(2000.0)),
        std::bind(&QoSSubscriber::counter_timer_cb_, this));

    // get the reliability and the durability from parameters
    this->declare_parameter("reliability", "best_effort");
    qos_reliability_param_ = this->get_parameter("reliability").as_string();
    this->declare_parameter("durability", "volatile");
    qos_durability_param_ = this->get_parameter("durability").as_string();

    // QoS profile
    rclcpp::QoS qos_profile(20);

    // set the reliability
    if (qos_reliability_param_ == "reliable")
    {
        qos_profile.reliable();
    }
    else if (qos_reliability_param_ == "best_effort")
    {
        qos_profile.best_effort();
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid reliability parameter: " << qos_reliability_param_);
        rclcpp::shutdown();
    }

    // set the durability
    if (qos_durability_param_ == "volatile")
    {
        qos_profile.durability_volatile();
    }
    else if (qos_durability_param_ == "transient")
    {
        qos_profile.transient_local();
    }
    else
    {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Invalid durability parameter: " << qos_durability_param_);
        rclcpp::shutdown();
    }

    // auto qos_profile = rclcpp::QoS(20).reliable().transient_local();
    // auto qos_profile = rclcpp::QoS(20).reliable().durability_volatile();
    // auto qos_profile = rclcpp::QoS(20).best_effort().durability_volatile();
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "leia",
        qos_profile,
        std::bind(
            &QoSSubscriber::leia_callback,
            this,
            std::placeholders::_1));
    print_qos(subscriber_->get_actual_qos());
}

void QoSSubscriber::leia_callback(const std_msgs::msg::String::SharedPtr msg)
{

    RCLCPP_INFO_STREAM(this->get_logger(), "Received: " << msg->data);
}

void QoSSubscriber::counter_timer_cb_()
{
    // update the count
    count_++;
    if (count_ == 5)
    {
        // auto qos_profile = rclcpp::QoS(rclcpp::KeepAll()).reliable();
        // subscriber_ = this->create_subscription<std_msgs::msg::String>(
        //     "leia",
        //     qos_profile,
        //     std::bind(
        //         &QoSSubscriber::leia_callback,
        //         this,
        //         std::placeholders::_1));
        // print_qos(subscriber_->get_actual_qos());
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto qos_subscriber = std::make_shared<QoSSubscriber>("qos_subscriber");
    rclcpp::spin(qos_subscriber);
    rclcpp::shutdown();
}