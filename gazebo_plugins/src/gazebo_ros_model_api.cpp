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
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <gazebo/msgs/msgs.hh>

#include <gazebo_plugins/gazebo_ros_model_api.h>
#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

using namespace gazebo;

GazeboRosModelAPI::GazeboRosModelAPI() : ROSServer("gazebo") {
}

// Destructor
GazeboRosModelAPI::~GazeboRosModelAPI() {
}

void GazeboRosModelAPI::Load ( physics::ModelPtr _parent, sdf::ElementPtr _sdf ) {
    int argc = 0;
    ROSServer::Load(argc, NULL);

    // Store the pointer to the model
    this->parent_ = _parent;
    this->world_ = _parent->GetWorld();
    this->robot_namespace_ = parent_->GetName ();

    ROS_INFO_NAMED("model_api", "Starting GazeboRosModelAPI Plugin (ns = %s)", this->robot_namespace_.c_str());
    last_update_time_ = this->world_->SimTime();

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin (
                                 boost::bind ( &GazeboRosModelAPI::OnUpdate, this, _1 ) );

    this->AdvertiseServices();
}

void GazeboRosModelAPI::AdvertiseServices() {
    // Advertise step control service (for one-step simulation)
    std::string publish_joint_states_service_name("publish_joint_states");
    ros::AdvertiseServiceOptions publish_joint_states_aso =
            ros::AdvertiseServiceOptions::create<gazebo_msgs::PublishJointStates>(
                    publish_joint_states_service_name,
                    boost::bind(&GazeboRosModelAPI::PublishJointStates,this,_1,_2),
                    ros::VoidPtr(), &gazebo_queue_);
    this->publish_joint_states_service_ = nh_->advertiseService(publish_joint_states_aso);

    std::string joint_states_topic((boost::format("/%1%/%2%") % robot_namespace_ % "joint_states").str());
    ROS_INFO_NAMED("model_api", "PublishJointStates() will publish out-of-band to: %s", joint_states_topic.c_str());
    this->joint_state_publisher_ = nh_->advertise<sensor_msgs::JointState> (joint_states_topic, 1000 );
}

void GazeboRosModelAPI::OnUpdate ( const common::UpdateInfo & _info )
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("GazeboRosModelAPI::OnUpdate");
#endif

    common::Time current_time = this->world_->SimTime();
    if (current_time < last_update_time_) {
        ROS_WARN_NAMED("model_api", "Negative joint state update time difference detected.");
        last_update_time_ = current_time;
    }

    double seconds_since_last_update = ( current_time - last_update_time_ ).Double();
    if ( seconds_since_last_update > update_period_ ) {
#ifdef ENABLE_PROFILER
        IGN_PROFILE_BEGIN("GazeboRosModelAPI::OnUpdate");
#endif
        this->DoUpdate(_info);
#ifdef ENABLE_PROFILER
        IGN_PROFILE_END();
#endif
        last_update_time_+= common::Time ( update_period_ );
    }

}

void GazeboRosModelAPI::DoUpdate(const common::UpdateInfo& _info) {
    // TODO: Nothing to do here right now
    // Fill in with actions as this implementation progresses
    // ..
    // ..
}

bool GazeboRosModelAPI::PublishJointStates(
        gazebo_msgs::PublishJointStates::Request& req,
        gazebo_msgs::PublishJointStates::Response& res)
{
    gazebo::common::Time currentTime = world_->SimTime();
    ros::Time current_time = ros::Time(currentTime.sec, currentTime.nsec);
    const gazebo::physics::Joint_V& joints = this->parent_->GetJoints();

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = current_time;
    joint_state_msg.name.resize ( joints.size() );
    joint_state_msg.position.resize ( joints.size() );
    joint_state_msg.velocity.resize ( joints.size() );
    for ( int i = 0; i < joints.size(); i++ ) {
        physics::JointPtr joint = joints[i];
        double velocity = joint->GetMsgType() == gazebo::msgs::Joint::FIXED ? 0.0 : joint->GetVelocity( 0 );
        double position = joint->GetMsgType() == gazebo::msgs::Joint::FIXED ? 0.0 : joint->Position ( 0 );
        joint_state_msg.name[i] = joint->GetName();
        joint_state_msg.position[i] = position;
        joint_state_msg.velocity[i] = velocity;
    }
    joint_state_publisher_.publish ( joint_state_msg );

    res.success = true;
    ROS_INFO_NAMED("model_api", "Published (out-of-band) joint states ...");
    return true;
}

