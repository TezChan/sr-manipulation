/**
* @file   real_arm_node.cpp
* @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
* @date   Tue Jun 29 14:56:10 2010
*
* @brief Contains the main for the real arm. We start the publishers / subscribers in this node.
* They all share the same RealArm object, this way the subscriber can update the arm properties,
* while the publishers publish up to date data. The diagnostics and the other publisher are started
* in two different threads, to allow them to be published at different frequencies.
*
*
*/

#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <kdl/tree.hpp>

#include <boost/thread.hpp>
#include <boost/smart_ptr.hpp>

#include "sr_hand/sr_subscriber.h"
#include "sr_hand/sr_publisher.h"
#include "sr_hand/sr_diagnosticer.h"
//#include "shadowhand/shadowhand_config_server.h"
#include "sr_hand/hand/real_arm.h"

using namespace std;
using namespace ros;
using namespace KDL;
using namespace shadowrobot;
//using namespace shadowhand_config_server;

/////////////////////////////////
//           MAIN              //
/////////////////////////////////

void run_diagnotics(boost::shared_ptr<SRDiagnosticer> shadowhand_diag)
{
  while( ok() )
    shadowhand_diag->publish();
}

void run_publisher(boost::shared_ptr<SRPublisher> shadowhand_pub)
{
  while( ok() )
    shadowhand_pub->publish();
}


/**
* The main function initialises this ROS subscriber and sets the different callbacks.
* This ROS subscriber will listen for new commands and send them to
* the real robot.
*
* @param argc
* @param argv
*
* @return 0 on success
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "shadowarm");
  NodeHandle n;

  boost::shared_ptr<RealArm> real_arm( new RealArm() );
  boost::shared_ptr<SRSubscriber> shadowhand_subscriber;

  boost::shared_ptr<SRPublisher> shadowhand_pub( new SRPublisher(real_arm));
  boost::shared_ptr<SRDiagnosticer> shadowhand_diag( new SRDiagnosticer(real_arm, sr_arm_hardware));

  // gets the location of the robot description on the parameter server
  string full_param_name;
  n.searchParam("robot_description",full_param_name);

  string robot_desc_string;
  n.param(full_param_name, robot_desc_string, string());
  Tree tree;
  if (!kdl_parser::treeFromString(robot_desc_string, tree))
  {
    ROS_ERROR("Failed to construct kdl tree");
  }
  else
  {
    ROS_DEBUG("kdl tree loaded!");
  }

  //  ShadowhandConfigServer shadowhand_config_server;

  if (tree.getNrOfSegments() == 0)
  {
    ROS_WARN("ShadowHand subscriber got an empty tree and cannot do inverse kinematics");
    shadowhand_subscriber = boost::shared_ptr<SRSubscriber>(new SRSubscriber(real_arm));
  }
  else if (tree.getNrOfSegments() == 1)
  {
    ROS_WARN("ShadowHand subscriber got an empty tree and cannot do inverse kinematics");

    shadowhand_subscriber = boost::shared_ptr<SRSubscriber>(new SRSubscriber(real_arm));
  }
  else
  {
    shadowhand_subscriber = boost::shared_ptr<SRSubscriber>(new SRSubscriber(real_arm, tree));
  }

  boost::thread thrd1( boost::bind( &run_diagnotics, shadowhand_diag ));
  boost::thread thrd2( boost::bind( &run_publisher, shadowhand_pub ));
  thrd1.join();
  thrd2.join();

  return 0;
}