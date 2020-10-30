//
// Created by safdar on 10/28/20.
//

#include <gazebo_ros/ros_server.h>
#include <thread>

namespace gazebo {

    ROSServer::ROSServer() : stop_(false), node_name_("~") {}
    ROSServer::ROSServer(const std::string& node_name) : stop_(false), node_name_(node_name) {}

    void ROSServer::Load(int argc, char** argv) {
        // connect to sigint event
        sigint_event_ = gazebo::event::Events::ConnectSigInt(boost::bind(&ROSServer::shutdownSignal,this));

        // setup ros related
        if (!ros::isInitialized()) {
            ros::init(argc, argv, "gazebo", ros::init_options::NoSigintHandler);
        } else {
            ROS_WARN_NAMED("api_plugin",
                            "Something other than this gazebo_ros_api plugin started ros::init(...), command line arguments may not be parsed properly.");
        }

        // check if the ros master is available - required
        while(!ros::master::check())
        {
            ROS_WARN_STREAM_NAMED("api_plugin","No ROS master - start roscore to continue...");
            // wait 0.5 second
            // can't use ROS Time here b/c node handle is not yet initialized
            std::this_thread::sleep_for(std::chrono::microseconds(500*1000));

            if(stop_)
            {
                ROS_WARN_STREAM_NAMED("api_plugin","Canceled loading Gazebo ROS API plugin by sigint event");
                return;
            }
        }

        nh_.reset(new ros::NodeHandle(node_name_)); // advertise topics and services in this node's namespace

        // Built-in multi-threaded ROS spinning
        async_ros_spin_.reset(new ros::AsyncSpinner(0)); // will use a thread for each CPU core
        async_ros_spin_->start();

        /// \brief setup custom callback queue
        gazebo_callback_queue_thread_.reset(new boost::thread( &ROSServer::gazeboQueueThread, this) );

    }

    void ROSServer::gazeboQueueThread()
    {
        static const double timeout = 0.001;
        while (nh_->ok())
        {
            gazebo_queue_.callAvailable(ros::WallDuration(timeout));
        }
    }

    void ROSServer::shutdownSignal()
    {
        ROS_DEBUG_STREAM_NAMED("api_plugin","shutdownSignal() recieved");
        stop_ = true;
    }

    ROSServer::~ROSServer() {
        // Stop the multi threaded ROS spinner
        async_ros_spin_->stop();
        ROS_DEBUG_STREAM_NAMED("api_plugin","Async ROS Spin Stopped");

        // Shutdown the ROS node
        nh_->shutdown();
        ROS_DEBUG_STREAM_NAMED("api_plugin","Node Handle Shutdown");

        // Shutdown ROS queue
        gazebo_callback_queue_thread_->join();
        ROS_DEBUG_STREAM_NAMED("api_plugin","Callback Queue Joined");
    }
}
