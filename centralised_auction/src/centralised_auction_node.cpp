#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "task_msgs/Task.h"
#include "task_msgs/TaskArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "waypoint_follower_msgs/Waypoint.h"
#include "waypoint_follower_msgs/WaypointStamped.h"
#include "waypoint_follower_msgs/WaypointArray.h"

#include "centralised_auction/sequential_auction.h"

using std::vector;
using std::cout;
using std::endl;
using std::string;
using task_msgs::Task;
using task_msgs::TaskArray;
using geometry_msgs::Pose;
using geometry_msgs::PoseArray;
using waypoint_follower_msgs::Waypoint;
using waypoint_follower_msgs::WaypointStamped;
using waypoint_follower_msgs::WaypointArray;


vector<Task> unallocated_tasks;
vector<Pose> robot_poses;

vector<TaskArray> allocated_tasks;

vector<ros::Publisher> pubs;
int num_robots;
int seq = 0;
string prefix;
bool return_home;

/**************************************************
 * Helper functions
 **************************************************/
 
// Converts from TaskArray format to PoseArray format.
PoseArray taskArrayToPoseArray( TaskArray ta ){
  PoseArray pa;
  pa.header.seq      = seq++;
  pa.header.stamp    = ros::Time::now();
  pa.header.frame_id = "map";
  for( int i=0; i<ta.array.size(); i++ ){
    pa.poses.push_back( ta.array[i].pose );
  }
  return pa;
}

// Converts from TaskArray format to WaypointArray format.
WaypointArray taskArrayToWaypointArray( TaskArray ta ){
  WaypointArray wa;
  WaypointStamped ws;
  ws.header.seq      = seq++;
  ws.header.stamp    = ros::Time::now();
  ws.header.frame_id = "map";
  Waypoint w;
  w.type = 5;
  for( int i=0; i<ta.array.size(); i++ ){
    w.id   = i;
    w.pose = ta.array[i].pose;
    ws.waypoint = w;
    wa.waypoints.push_back(ws);
  }
  return wa;
}


/**************************************************
 * Publisher functions
 **************************************************/
 
// Publishes the allocations.
void publishAllocations(){
  for( int i=0; i<num_robots; i++ ){
    TaskArray ta = allocated_tasks[i];
    //PoseArray pa = taskArrayToPoseArray(ta);
    //pubs[i].publish( pa );
    WaypointArray wa = taskArrayToWaypointArray(ta);
    if( return_home ){
      WaypointStamped ws;
      ws.header.seq         = seq++;
      ws.header.stamp       = ros::Time::now();
      ws.header.frame_id    = "map";
      ws.waypoint.type      = 7;
      ws.waypoint.completed = false;
      ws.waypoint.id        = ta.array.size() + i;
      ws.waypoint.pose      = robot_poses[i];
      wa.waypoints.push_back(ws);
    }
    pubs[i].publish( wa );
  }
}

/**************************************************
 * 
 **************************************************/
 
void allocateTasks(){
  if( num_robots <= 0 ){
    cout << "No robots specified." << endl;
    return;
  }
  if( unallocated_tasks.size() <= 0 ){
    cout << "No tasks specified." << endl;
    return;
  }

  // Allocate them all to the first robot.
  //for( int i=0; i<unallocated_tasks.size(); i++ ){
  //  Task t = unallocated_tasks[i];
  //  int r = i % num_robots;
  //  allocated_tasks[r].array.push_back(t);
  //}
  
  SequentialAuction solver(unallocated_tasks, robot_poses);
  solver.use_least_contested_bid = true;
  if( return_home ) solver.return_home             = true;
  allocated_tasks = solver.allocateTasks();
  

}


/**************************************************
 * Initialising functions
 **************************************************/
 
//void loadSubs(ros::NodeHandle n){                                 
//}

void loadPubs(ros::NodeHandle n){
  for( int i=0; i<num_robots; i++ ){
    std::ostringstream strs;
    strs << prefix;   
    strs << i;
    strs << "/tasks";
    //pubs.push_back( n.advertise<PoseArray>(strs.str(), 100) );
    pubs.push_back( n.advertise<WaypointArray>(strs.str(), 100) );
  }
}

// Loads parameters from the launch file. Uses default values if any values are
// missing.
void loadParams(ros::NodeHandle n_priv){
  vector<double> waypoint_values;    //position (x,y,z) and orientation (x,y,z,w)
  vector<double> robot_values;       //position (x,y,z)
  
  // Set default parameters.
  double default_waypoint_values[] = {};
  double default_robot_values[] = {};
  string default_prefix = "robot";
  bool default_return_home = true;
  
  n_priv.param("return_home", return_home, default_return_home);
  n_priv.param("prefix", prefix, default_prefix);
  
  // Check parameter server to override defaults.
  XmlRpc::XmlRpcValue v;
  if( n_priv.getParam("tasks", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        waypoint_values.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        waypoint_values.push_back(d);
      }
    }
  } else {
    waypoint_values.assign(default_waypoint_values, default_waypoint_values + 
                      sizeof(default_waypoint_values) / sizeof(default_waypoint_values[0]) );
  }
  // Convert waypoint values into waypoints.
  if( waypoint_values.size() % 7 != 0 ){
    cout << "INCORRECT NUMBER OF WAYPOINT VALUES" << endl;
    return;
  }
  for( int i=0; i<waypoint_values.size(); i+=7 ) {
    Task t;
    t.pose.position.x = waypoint_values[i];
    t.pose.position.y = waypoint_values[i+1];
    t.pose.position.z = waypoint_values[i+2];
    t.pose.orientation.x = waypoint_values[i+3];
    t.pose.orientation.y = waypoint_values[i+4];
    t.pose.orientation.z = waypoint_values[i+5];
    t.pose.orientation.w = waypoint_values[i+6];
    unallocated_tasks.push_back(t);
  }
  
  // Check parameter server to override defaults.
  //XmlRpc::XmlRpcValue v;
  if( n_priv.getParam("robots", v) ){
    ROS_ASSERT(v.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for(int i=0; i < v.size(); i++)  {
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeDouble ) {
        robot_values.push_back(v[i]);
      }
      if( v[i].getType() == XmlRpc::XmlRpcValue::TypeInt ) {
        int d = v[i];
        robot_values.push_back(d);
      }
    }
  } else {
    robot_values.assign(default_robot_values, default_robot_values + 
                      sizeof(default_robot_values) / sizeof(default_robot_values[0]) );
  }
  // Convert waypoint values into waypoints.
  if( robot_values.size() % 3 != 0 ){
    cout << "INCORRECT NUMBER OF ROBOT VALUES" << endl;
    return;
  }
  for( int i=0; i<robot_values.size(); i+=3 ) {
    Pose p;
    p.position.x = robot_values[i];
    p.position.y = robot_values[i+1];
    p.position.z = robot_values[i+2];
    robot_poses.push_back(p);
  }
  num_robots = robot_poses.size();
  for( int i=0; i<num_robots; i++ ){
    TaskArray ta;
    allocated_tasks.push_back( ta );
  }
  
}


/**************************************************
 * Main
 **************************************************/
int main(int argc, char** argv){
  ros::init(argc, argv, "centralised_auction");
  ros::NodeHandle n;
  ros::NodeHandle n_priv("~");
  
  cout << "IT HAS BEGUN" << endl;
  
  loadParams(n_priv);
  //loadSubs(n);
  loadPubs(n);
  
  allocateTasks();
  publishAllocations();
    
  ros::Rate r(1);
  while( ros::ok() ){
    publishAllocations();
    ros::spinOnce();
    r.sleep();
  }
  
  return 0;
};
