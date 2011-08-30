/**
 * @file   virtual_shadowhand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:50:42 2010
 *
*
* Copyright 2011 Shadow Robot Company Ltd.
*
* This program is free software: you can redistribute it and/or modify it
* under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 2 of the License, or (at your option)
* any later version.
*
* This program is distributed in the hope that it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along
* with this program.  If not, see <http://www.gnu.org/licenses/>.
*
 * @brief
 *
 *
 */

#include "sr_hand/hand/virtual_shadowhand.h"

#include <time.h>
#include <ros/ros.h>

#ifdef GAZEBO
#include <std_msgs/Float64.h>
#endif

namespace shadowrobot
{
  VirtualShadowhand::VirtualShadowhand() :
    SRArticulatedRobot()
  {
#ifdef GAZEBO
    ROS_INFO("This ROS interface is built for Gazebo.");
    //initialises the subscriber to the Gazebo joint_states messages
    std::string prefix;
    std::string searched_param;
    n_tilde = ros::NodeHandle("~");

    n_tilde.searchParam("gazebo_joint_states_prefix", searched_param);
    n_tilde.param(searched_param, prefix, std::string());
    std::string full_topic = prefix + "joint_states";
    gazebo_subscriber = node.subscribe(full_topic, 2, &VirtualShadowhand::gazeboCallback, this);
#else
    ROS_INFO("This ROS interface is not built for Gazebo.");
#endif

    srand(time(NULL));
    initializeMap();
  }

  VirtualShadowhand::~VirtualShadowhand()
  {
  }

  void VirtualShadowhand::initializeMap()
  {
    joints_map_mutex.lock();
    parameters_map_mutex.lock();
    controllers_map_mutex.lock();

    JointData tmpData;
    JointData tmpDataZero;
    JointControllerData tmpController;
    tmpDataZero.isJointZero = 1;
    tmpDataZero.max = 180.0;

#ifdef GAZEBO
    std::string topic_prefix = "/";
    std::string topic_suffix = "/command";
    std::string full_topic = "";
#endif

    joints_map["FFJ0"] = tmpDataZero;
    controllers_map["FFJ0"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "ffdistal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    int tmp_index = 0;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["FFJ1"] = tmpData;
    controllers_map["FFJ1"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "ffmiddle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["FFJ2"] = tmpData;
    controllers_map["FFJ2"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "ffproximal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["FFJ3"] = tmpData;
    controllers_map["FFJ3"] = tmpController;
    tmpData.min = -25.0;
    tmpData.max = 25.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "ffknuckle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["FFJ4"] = tmpData;
    controllers_map["FFJ4"] = tmpController;

    joints_map["MFJ0"] = tmpDataZero;
    controllers_map["MFJ0"] = tmpController;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "mfdistal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["MFJ1"] = tmpData;
    controllers_map["MFJ1"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "mfmiddle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["MFJ2"] = tmpData;
    controllers_map["MFJ2"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "mfproximal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["MFJ3"] = tmpData;
    controllers_map["MFJ3"] = tmpController;
    tmpData.min = -25.0;
    tmpData.max = 25.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "mfknuckle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["MFJ4"] = tmpData;
    controllers_map["MFJ4"] = tmpController;

    joints_map["RFJ0"] = tmpDataZero;
    controllers_map["RFJ0"] = tmpController;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "rfdistal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["RFJ1"] = tmpData;
    controllers_map["RFJ1"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "rfmiddle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["RFJ2"] = tmpData;
    controllers_map["RFJ2"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "rfproximal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["RFJ3"] = tmpData;
    controllers_map["RFJ3"] = tmpController;
    tmpData.min = -25.0;
    tmpData.max = 25.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "rfknuckle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["RFJ4"] = tmpData;
    controllers_map["RFJ4"] = tmpController;

    joints_map["LFJ0"] = tmpDataZero;
    controllers_map["LFJ0"] = tmpController;
    tmpData.min = 0.0;
    tmpData.max = 90.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "lfdistal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["LFJ1"] = tmpData;
    controllers_map["LFJ1"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "lfmiddle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["LFJ2"] = tmpData;
    controllers_map["LFJ2"] = tmpController;
#ifdef GAZEBO
    full_topic = topic_prefix + "lfproximal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["LFJ3"] = tmpData;
    controllers_map["LFJ3"] = tmpController;
    tmpData.min = -25.0;
    tmpData.max = 25.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "lfknuckle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["LFJ4"] = tmpData;
    controllers_map["LFJ4"] = tmpController;
    tmpData.min = 0.0;
    tmpData.max = 45.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "lfmetacarpal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["LFJ5"] = tmpData;
    controllers_map["LFJ5"] = tmpController;

    tmpData.min = 0.0;
    tmpData.max = 90.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "thdistal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["THJ1"] = tmpData;
    controllers_map["THJ1"] = tmpController;
    tmpData.min = -30.0;
    tmpData.max = 30.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "thmiddle_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["THJ2"] = tmpData;
    controllers_map["THJ2"] = tmpController;
    tmpData.min = -15.0;
    tmpData.max = 15.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "thhub_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["THJ3"] = tmpData;
    controllers_map["THJ3"] = tmpController;
    tmpData.min = 0.0;
    tmpData.max = 75.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "thproximal_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["THJ4"] = tmpData;
    controllers_map["THJ4"] = tmpController;
    tmpData.min = -60.0;
    tmpData.max = 60.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "thbase_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["THJ5"] = tmpData;
    controllers_map["THJ5"] = tmpController;

    tmpData.min = -30.0;
    tmpData.max = 40.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "palm_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["WRJ1"] = tmpData;
    controllers_map["WRJ1"] = tmpController;
    tmpData.min = -30.0;
    tmpData.max = 10.0;
#ifdef GAZEBO
    full_topic = topic_prefix + "wrist_controller" + topic_suffix;
    gazebo_publishers.push_back(node.advertise<std_msgs::Float64>(full_topic, 2));
    tmp_index ++;
    tmpData.publisher_index = tmp_index;
#endif
    joints_map["WRJ2"] = tmpData;
    controllers_map["WRJ2"] = tmpController;

    parameters_map["d"] = PARAM_d;
    parameters_map["i"] = PARAM_i;
    parameters_map["p"] = PARAM_p;
    parameters_map["target"] = PARAM_target;
    parameters_map["sensor"] = PARAM_sensor;

    parameters_map["valve"] = PARAM_valve;
    parameters_map["dead"] = PARAM_deadband;
    parameters_map["deadband"] = PARAM_deadband;
    parameters_map["imax"] = PARAM_imax;
    parameters_map["offset"] = PARAM_output_offset;
    parameters_map["shift"] = PARAM_shift;
    parameters_map["max"] = PARAM_output_max;

    //! the parameters for the motors
    parameters_map["motor_maxforce"] = PARAM_motor_maxforce;
    parameters_map["motor_safeforce"] = PARAM_motor_safeforce;

    parameters_map["force_p"] = PARAM_force_p;
    parameters_map["force_i"] = PARAM_force_i;
    parameters_map["force_d"] = PARAM_force_d;

    parameters_map["force_imax"] = PARAM_force_imax;
    parameters_map["force_out_shift"] = PARAM_force_out_shift;
    parameters_map["force_deadband"] = PARAM_force_deadband;
    parameters_map["force_offset"] = PARAM_force_offset;

    parameters_map["sensor_imax"] = PARAM_sensor_imax;
    parameters_map["sensor_out_shift"] = PARAM_sensor_out_shift;
    parameters_map["sensor_deadband"] = PARAM_sensor_deadband;
    parameters_map["sensor_offset"] = PARAM_sensor_offset;
    parameters_map["max_temp"] = PARAM_max_temperature;
    parameters_map["max_temperature"] = PARAM_max_temperature;
    parameters_map["max_current"] = PARAM_max_current;

    controllers_map_mutex.unlock();
    parameters_map_mutex.unlock();
    joints_map_mutex.unlock();
  }

  short VirtualShadowhand::sendupdate( std::string joint_name, double target )
  {
    joints_map_mutex.lock();

    JointsMap::iterator iter = joints_map.find(joint_name);
#ifdef GAZEBO
    std_msgs::Float64 target_msg;
#endif

    //not found
    if( iter == joints_map.end() )
    {
      ROS_DEBUG("Joint %s not found", joint_name.c_str());

      joints_map_mutex.unlock();
      return -1;
    }

    //if joint 0, send 1/2 of the target to joint 1 and other half to
    //2;
    if( iter->second.isJointZero == 1 )
    {
      //push target and position to the given target for Joint 0
      JointData tmpData0 = JointData(iter->second);
      if( target < tmpData0.min )
        target = tmpData0.min;
      if( target > tmpData0.max )
        target = tmpData0.max;

#ifndef GAZEBO
      tmpData0.position = target;
#endif
      tmpData0.target = target;

      joints_map[joint_name] = tmpData0;

      ++iter;
      JointData tmpData1 = JointData(iter->second);
#ifdef GAZEBO
      //gazebo targets are in radians
      target_msg.data = toRad( target / 2.0 );
      gazebo_publishers[tmpData1.publisher_index].publish(target_msg);
      ros::spinOnce();
#else
      tmpData1.position = target / 2.0;
#endif
      tmpData1.target = target / 2.0;

      joints_map[iter->first] = tmpData1;

      ++iter;
      JointData tmpData2 = JointData(iter->second);
#ifdef GAZEBO
      //gazebo targets are in radians
      target_msg.data = toRad( target / 2.0 );
      gazebo_publishers[tmpData2.publisher_index].publish(target_msg);
      ros::spinOnce();
#else
      tmpData2.position = target / 2.0;
#endif
      tmpData2.target = target / 2.0;

      joints_map[iter->first] = tmpData2;

      joints_map_mutex.unlock();
      return 0;
    }

    //joint found
    JointData tmpData(iter->second);

    if( target < tmpData.min )
      target = tmpData.min;
    if( target > tmpData.max )
      target = tmpData.max;

#ifdef GAZEBO
    //gazebo targets are in radians
    target_msg.data = toRad(target);
    gazebo_publishers[tmpData.publisher_index].publish(target_msg);
    ros::spinOnce();
#else
    tmpData.position = target;
#endif
    tmpData.target = target;

    joints_map[joint_name] = tmpData;

    joints_map_mutex.unlock();
    return 0;
  }

  JointData VirtualShadowhand::getJointData( std::string joint_name )
  {
    joints_map_mutex.lock();

    JointsMap::iterator iter = joints_map.find(joint_name);

    //joint found
    if( iter != joints_map.end() )
    {
      //return the position
      iter->second.temperature = ((double)(rand() % 100) / 100.0);
      iter->second.current = ((double)(rand() % 100) / 100.0);
      iter->second.force = ((double)(rand() % 100) / 100.0);

      JointData tmp = JointData(iter->second);

      joints_map_mutex.unlock();
      return tmp;
    }

    ROS_ERROR("Joint %s not found.", joint_name.c_str());
    JointData noData;
    joints_map_mutex.unlock();
    return noData;
  }

  SRArticulatedRobot::JointsMap VirtualShadowhand::getAllJointsData()
  {
    joints_map_mutex.lock();

    for( JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
    {
      JointData tmpData = it->second;
      tmpData.temperature = ((double)(rand() % 100) / 100.0);
      tmpData.current = ((double)(rand() % 100) / 100.0);
      tmpData.force = ((double)(rand() % 100) / 100.0);
      tmpData.jointIndex = 0;
      tmpData.flags = "";

      joints_map[it->first] = tmpData;
    }

    JointsMap tmp_map = JointsMap(joints_map);
    joints_map_mutex.unlock();
    return tmp_map;
  }

  short VirtualShadowhand::setContrl( std::string contrlr_name, JointControllerData ctrlr_data )
  {
    controllers_map_mutex.lock();

    ControllersMap::iterator iter = controllers_map.find(contrlr_name);

    //joint found
    if( iter != controllers_map.end() )
    {
      controllers_map[iter->first] = ctrlr_data;
    }
    else
    {
      ROS_ERROR("Controller %s not found", contrlr_name.c_str());
    }

    controllers_map_mutex.unlock();
    return 0;
  }

  JointControllerData VirtualShadowhand::getContrl( std::string contrlr_name )
  {
    controllers_map_mutex.lock();
    ControllersMap::iterator iter = controllers_map.find(contrlr_name);

    //joint found
    if( iter != controllers_map.end() )
    {
      JointControllerData tmp = JointControllerData(iter->second);
      controllers_map_mutex.unlock();
      return tmp;
    }

    ROS_ERROR("Controller %s not found", contrlr_name.c_str() );
    JointControllerData no_result;
    controllers_map_mutex.unlock();
    return no_result;
  }

  short VirtualShadowhand::setConfig( std::vector<std::string> myConfig )
  {
    ROS_WARN("The set config function is not implemented in the virtual shadowhand.");
    return 0;
  }

  void VirtualShadowhand::getConfig( std::string joint_name )
  {
    ROS_WARN("The get config function is not implemented in the virtual shadowhand.");
  }

  std::vector<DiagnosticData> VirtualShadowhand::getDiagnostics()
  {
    joints_map_mutex.lock();
    std::vector<DiagnosticData> returnVect;

    for( JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it )
    {
      DiagnosticData tmpDiag;
      tmpDiag.joint_name = it->first;
      tmpDiag.level = 0;
      tmpDiag.flags = "";
      tmpDiag.target_sensor_num = 0;
      tmpDiag.target = it->second.target;
      tmpDiag.position_sensor_num = 0;
      tmpDiag.position = it-> second.position;

      returnVect.push_back(tmpDiag);
    }

    joints_map_mutex.unlock();
    return returnVect;
  }

#ifdef GAZEBO
  void VirtualShadowhand::gazeboCallback(const sensor_msgs::JointStateConstPtr& msg)
  {
    joints_map_mutex.lock();

    //loop on all the names in the joint_states message
    for(unsigned int index = 0; index < msg->name.size(); ++index)
    {
      std::string joint_name = msg->name[index];
      JointsMap::iterator iter = joints_map.find(joint_name);
      //not found => can be a joint from the arm / hand
      if(iter == joints_map.end())
        continue;

      //joint found
      JointData tmpData(iter->second);

      tmpData.position = toDegrees(msg->position[index]);
      tmpData.force = msg->effort[index];

      joints_map[joint_name] = tmpData;
    }

    //push the sum of J1+J2 to the J0s
    for(JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
    {
      JointData tmpData = it->second;
      if( tmpData.isJointZero == 1 )
      {
        std::string joint_name = it->first;
        double position = 0.0;

        //get the position from joint 1
        ++it;
        JointData tmpData1 = JointData(it->second);
        position += tmpData1.position;

        //get the position from joint 2
        ++it;
        JointData tmpData2 = JointData(it->second);
        position += tmpData2.position;

        tmpData.position = position;

        joints_map[joint_name] = tmpData;
      }
    }

    joints_map_mutex.unlock();
  }

#endif
} //end namespace


/* For the emacs weenies in the crowd.
Local Variables:
   c-basic-offset: 2
End:
*/