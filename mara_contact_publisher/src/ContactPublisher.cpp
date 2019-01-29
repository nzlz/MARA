/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo_msgs/msg/contact_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/wrench.hpp>

#include <gazebo/gazebo_client.hh>

#include <iostream>

rclcpp::Publisher<gazebo_msgs::msg::ContactState> contacts_pub ;

// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
  gazebo_msgs::ContactState contact;
  geometry_msgs::Vector3 position;
  geometry_msgs::Vector3 normals;
  geometry_msgs::Wrench total_wrench;


  if (_msg->contact_size() > 0){
    if (_msg->contact(0).collision1() != "ground_plane::link::collision" && _msg->contact(0).collision2() != "ground_plane::link::collision"){
      contact.collision1_name = _msg->contact(0).collision1();
      contact.collision2_name = _msg->contact(0).collision2();

      for (unsigned int j = 0; j <  _msg->contact(0).position_size(); ++j){
        contact.collision1_name = _msg->contact(0).collision1();
        contact.collision2_name = _msg->contact(0).collision2();

        position.x =  _msg->contact(0).position(j).x();
        position.y =  _msg->contact(0).position(j).y();
        position.z =  _msg->contact(0).position(j).z();
        contact.contact_positions.push_back(position);

        normals.x =  _msg->contact(0).normal(j).x();
        normals.y =  _msg->contact(0).normal(j).y();
        normals.z =  _msg->contact(0).normal(j).z();
        contact.contact_normals.push_back(normals);

        contact.depths.push_back(static_cast<float>(_msg->contact(0).depth(0)));
      }
    }
  }
  contacts_pub.publish(contact);

}

/////////////////////////////////////////////////
int main(int argc, char ** argv)
{
  // Load gazebo
  gazebo::client::setup(argc, argv);

  rclcpp::init(argc, argv);
  ros_node = gazebo_ros::Node::Get();

	contacts_pub = ros_node.advertise<gazebo_msgs::ContactState>("gazebo_contacts", 1);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", cb);

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}