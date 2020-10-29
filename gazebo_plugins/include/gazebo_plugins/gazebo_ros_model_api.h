/*
 * Copyright (c) 2013, Markus Bader <markus.bader@tuwien.ac.at>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY Antons Rebguns <email> ''AS IS'' AND ANY
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL Antons Rebguns <email> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 *  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/


#ifndef JOINT_STATE_SERVICE_PLUGIN_HH
#define JOINT_STATE_SERVICE_PLUGIN_HH

#include <stdio.h>
#include <boost/bind.hpp>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo_msgs/PublishJointStates.h"

// ROS
#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>

#include <gazebo_ros/ros_server.h>

// Usage in URDF:
//   <gazebo>
//       <plugin name="ros_model_api" filename="libgazebo_ros_model_api.so">
// 		    <robotNamespace>/...</robotNamespace>
// 		    <updateRate>100.0</updateRate>
// 		    <alwaysOn>true</alwaysOn>
//       </plugin>
//   </gazebo>


namespace gazebo {
class GazeboRosModelAPI : public ModelPlugin, public ROSServer {
public:
    GazeboRosModelAPI();
    ~GazeboRosModelAPI();
    void Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf );
    void OnUpdate ( const common::UpdateInfo & _info );
    void DoUpdate(const common::UpdateInfo& _info);

protected:
    void AdvertiseServices();
    bool PublishJointStates(gazebo_msgs::PublishJointStates::Request &req, gazebo_msgs::PublishJointStates::Response &res);

protected:
    event::ConnectionPtr updateConnection;
    physics::WorldPtr world_;
    physics::ModelPtr parent_;

    // ROS STUFF
    std::string robot_namespace_;
    ros::ServiceServer publish_joint_states_service_;
    ros::Publisher joint_state_publisher_;

    // Update Rate
    double update_rate_;
    double update_period_;
    common::Time last_update_time_;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN ( GazeboRosModelAPI )
}

#endif //JOINT_STATE_SERVICE_PLUGIN_HH

