//
// Created by safdar on 10/28/20.
//

#ifndef GAZEBO_ROS_PKGS_ROS_SERVER_H
#define GAZEBO_ROS_PKGS_ROS_SERVER_H

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <iostream>

#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>

// ROS
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#include <boost/shared_ptr.hpp>


namespace gazebo {
    class ROSServer {
    public:
        ROSServer();
        virtual void Load(int argc, char** argv);
        virtual ~ROSServer();

    protected:
        /// \brief ros queue thread for this node
        void gazeboQueueThread();
        /// \bried Detect if sig-int shutdown signal is recieved
        void shutdownSignal();

    protected:
        boost::shared_ptr<ros::NodeHandle> nh_;
        ros::CallbackQueue gazebo_queue_;
        boost::shared_ptr<boost::thread> gazebo_callback_queue_thread_;

        // ROS comm
        boost::shared_ptr<ros::AsyncSpinner> async_ros_spin_;

        // detect if sigint event occurs
        bool stop_;
        gazebo::event::ConnectionPtr sigint_event_;
    };
}




#endif //GAZEBO_ROS_PKGS_ROS_SERVER_H
