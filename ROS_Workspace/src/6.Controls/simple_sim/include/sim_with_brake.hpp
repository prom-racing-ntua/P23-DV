#include "sim_actuator_modelling.hpp"

namespace sim
{


class sim_node: public rclcpp::Node
{
    public:
    sim_node();
    ~sim_node() {std::cout<<max_time<<std::endl;}
    private:
    rclcpp::Publisher<custom_msgs::msg::PoseMsg>::SharedPtr pub_pose;
    rclcpp::Publisher<custom_msgs::msg::LocalMapMsg>::SharedPtr pub_map;
    rclcpp::Publisher<custom_msgs::msg::WaypointsMsg>::SharedPtr pub_way;
    rclcpp::Publisher<custom_msgs::msg::VelEstimation>::SharedPtr pub_vel;
    rclcpp::Publisher<custom_msgs::msg::TxSystemState>::SharedPtr pub_syst;
    rclcpp::Publisher<custom_msgs::msg::RxVehicleSensors>::SharedPtr pub_sens;
    rclcpp::Publisher<custom_msgs::msg::RxSteeringAngle>::SharedPtr pub_steer;
    rclcpp::Publisher<custom_msgs::msg::RxWheelSpeed>::SharedPtr pub_wheel;
    rclcpp::Publisher<custom_msgs::msg::AutonomousStatus>::SharedPtr pub_aut;
    rclcpp::Publisher<custom_msgs::msg::MissionSelection>::SharedPtr pub_miss;
    rclcpp::Subscription<custom_msgs::msg::TxControlCommand>::SharedPtr sub_comm;

    State state;
    Constants constants;

    std::vector<Cone> seen_cones;
    std::vector<Cone> unseen_cones;
    std::vector<double> torques;
    std::vector<double> steering;

    void timer_callback();
    void command_callback(const custom_msgs::msg::TxControlCommand::SharedPtr msg);
    rclcpp::TimerBase::SharedPtr timer_;
    bool lap_change()const;

    void pubs_and_subs();
    void parameter_load();

    double max_time = 0;

    // Actuator modelling
    steeringActuation steering_model;
    double steering_response, steering_velocity;

    brakeActuation brake_model;
    double brake_response, brake_velocity;

    motorActuation motor_model;
    double motor_response;

    double perception_range;

    std::ofstream log;
    long long int global_idx;
    double steering_dead_time;
    double motor_dead_time;
    double brake_dead_time;
    int st_d_ticks;
    int mot_d_ticks;
    int br_d_ticks;
    double last_d, last_d2;
    int idx_of_last_lap;
    int sent;
    int discipline;
    int is_end;
    float brake_press;
    int total_doo;
};
std::ostream &operator<<(std::ostream &out, const State &a);
}