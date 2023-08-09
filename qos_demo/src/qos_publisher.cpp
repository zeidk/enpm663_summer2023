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

    // publisher
    // auto qos_profile = rclcpp::QoS(20).reliable().transient_local();
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