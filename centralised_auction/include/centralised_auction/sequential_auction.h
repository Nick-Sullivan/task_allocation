#ifndef SEQUENTIAL_AUCTION_H
#define SEQUENTIAL_AUCTION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "task_msgs/Task.h"
#include "task_msgs/TaskArray.h"
#include "geometry_msgs/Pose.h"

using std::cout;
using std::endl;
using std::vector;
using task_msgs::Task;
using task_msgs::TaskArray;
using geometry_msgs::Pose;


class SequentialAuction {
  public:
    SequentialAuction(vector<Task> unallocated_tasks,
                      vector<Pose> robot_poses);
    vector<TaskArray> allocateTasks();
    
  private:
    // These variables are unchanged through allocation.
    bool              return_home;
    int               num_robots;
    int               num_tasks;
    vector<Pose>      robot_poses;
    vector<Task>      tasks;
    // Working variables for alloction.
    vector<int>              unalloc;
    vector< vector<int> >    paths;       //consider changing to lists for efficiency
    vector<double>           path_costs;
    vector< vector<double> > bids;
    
    // Output variable from allocation.
    vector<TaskArray> allocations;
    
    void formOutput();
    void processWinner(int winning_robot, int winning_task);
    void selectWinner(int &winning_robot, int &winning_task);
    void prepareAllocations();
    void calculateAllBids();
    void calculateBids(int robot_num);
    double insertTask( int robot_num, int unalloc_id, vector<int>& new_path);
    double calculatePathCost( int robot_num, vector<int> path );
    void printPaths();
    void printPath( vector<int> path );
    void printBids();

};


#endif //SEQUENTIAL_AUCTION_H
