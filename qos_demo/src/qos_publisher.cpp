#include <rclcpp/rclcpp.hpp>
#include "qos_publisher.hpp"

double QoSPublisher::rmw_time_to_seconds(const rmw_time_t &time)
{
    double result = static_cast<double>(time.sec);
    result += 1e-9 * time.nsec;
    return result;
}

void QoSPublisher::print_qos(const rclcpp::QoS &qos)
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

void QoSPublisher::publisher_timer_cb_()
{
    publish_();
}


void QoSPublisher::publish_()
{
    // update the message using count_
    msg_.data = std::to_string(count_) + ": Help me Obi-Wan Kenobi, you are my only hope.";
    RCLCPP_INFO_STREAM(this->get_logger(), msg_.data);
    publisher_->publish(msg_);

    count_++;
}

void QoSPublisher::initialize_()
{
    // initialize the count
    count_ = 0;
    // initialize the message
    msg_ = std_msgs::msg::String();
    // publisher timer
    publish_timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(2000.0)),
                                     std::bind(&QoSPublisher::publisher_timer_cb_, this));
    // publisher
    rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepAll()).best_effort();
    publisher_ = this->create_publisher<std_msgs::msg::String>("leia", qos_profile);
    print_qos(publisher_->get_actual_qos());
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto qos_publisher = std::make_shared<QoSPublisher>("qos_publisher");
    rclcpp::spin(qos_publisher);
    rclcpp::shutdown();
}