#include "lifecycle_mpc_node.hpp"


namespace mpc{
    mpc::CallbackReturn
        LifecycleMpcHandler::on_configure()
    {
        mpc_solver.mem = FORCESNLPsolver_internal_mem(0);
        if(!mpc_solver.known_track_){
            path_subscriber_ = create_subscription<custom_msgs::msg::WaypointsMsg>("waypoints", 10, std::bind(&LifecycleMpcHandler::path_callback, this, std::placeholders::_1));
        }
        else {
            ReadKnownTrack();
        }
        setSubscribers();
        // mpc_clock_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/node_freq_)),std::bind(&LifecycleMpcHandler::mpc_callback, this));
        // mpc_clock_->cancel();
        mpc_publisher_ = create_publisher<custom_msgs::msg::TxControlCommand>("control_commands", 10);

        RCLCPP_WARN(get_logger(), "\n-- MPC Configured!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    mpc::CallbackReturn
        LifecycleMpcHandler::on_activate()
    {
        /* Activate MPC Publisher */
        mpc_publisher_->on_activate();

        /* Activate MPC clock */
        mpc_clock_->cancel();
        mpc_clock_->reset();

        RCLCPP_WARN(get_logger(), "\n-- MPC Activated!");
        return mpc::CallbackReturn::SUCCESS;
    }

    mpc::CallbackReturn
        LifecycleMpcHandler::on_deactivate()
    {
        mpc_publisher_->on_deactivate();
        // mpc_clock_->cancel();
    
        RCLCPP_INFO(get_logger(), "\n-- MPC Node De-Activated");
        return mpc::CallbackReturn::SUCCESS;
    }

    mpc::CallbackReturn
        LifecycleMpcHandler::on_cleanup()
    {
        /* Should actually free up memory and such */
        
        mpc_publisher_.reset();
        RCLCPP_WARN(get_logger(), "\n-- MPC Un-Configured!");
        return mpc::CallbackReturn::SUCCESS;
    }

    mpc::CallbackReturn
        LifecycleMpcHandler::on_shutdown()
    {
        mpc_publisher_->on_deactivate();
        mpc_publisher_.reset();

        // mpc_clock_->cancel();

        /* Should actually free up memory and such */

        RCLCPP_INFO(get_logger(), "\n-- MPC Shut Down");

        return mpc::CallbackReturn::SUCCESS;
    }

    mpc::CallbackReturn
        LifecycleMpcHandler::on_error()
    {
        return mpc::CallbackReturn::SUCCESS;
    }

}

