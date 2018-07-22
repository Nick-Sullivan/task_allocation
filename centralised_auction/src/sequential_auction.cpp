#include "centralised_auction/sequential_auction.h"

/********************************************
 * Public functions (high level).
 ********************************************/
 
// Constructor.
SequentialAuction::SequentialAuction(vector<Task> unallocated_tasks,
                                     vector<Pose> robot_poses){
  this->tasks       = unallocated_tasks;
  this->num_tasks   = unallocated_tasks.size();
  this->robot_poses = robot_poses;
  this->num_robots  = robot_poses.size();
  return_home = false;
  use_least_contested_bid = false;
}

// The allocation procedure. Each iteration, selects a winner to allocate and
// recalculates bids. 
vector<TaskArray> SequentialAuction::allocateTasks(){
  int winning_robot, winning_task;
  prepareAllocations();
  calculateAllBids();
  while( !unalloc.empty() ){
    //cout << "The bids are: " << endl;
    //printBids();
    selectWinner(winning_robot, winning_task);
    //cout << "winner is: " << winning_robot << ", " << winning_task << endl;
    processWinner(winning_robot, winning_task);
    calculateBids(winning_robot);                                              
  }        
  cout << "Allocations: " << endl;
  printPaths();
  
  formOutput();
  return allocations;                                        
}

/********************************************
 * Allocation processing (medium level)
 ********************************************/
 
// Clears paths, sets the correct size for bidding matrix.
void SequentialAuction::prepareAllocations(){
  unalloc.clear();
  paths.clear();
  path_costs.clear();
  bids.clear();
  allocations.clear();
  for( int i=0; i<num_tasks; i++ ){
    unalloc.push_back(i);
  }
  for( int i=0; i<num_robots; i++ ){
    vector<int> v;
    paths.push_back( v );
    path_costs.push_back(0);
  }
  for( int i=0; i<num_robots; i++ ){
    vector<double> v;
    for( int j=0; j<num_tasks; j++ ){
      v.push_back(-1);
    }
    bids.push_back( v );
  }
}

void SequentialAuction::calculateAllBids(){
  for( int i=0; i<num_robots; i++ ){
    calculateBids( i );
  }
}

// Populates the bidding matrix with bids for the currently unallocated tasks.
void SequentialAuction::calculateBids( int robot_num ){
  for( int i=0; i<unalloc.size(); i++ ){
    //cout << "task " << unalloc[i] << endl;
    // Get the current path cost.
    double prev_cost = path_costs[robot_num];
    // Calculate the new path cost if we were to add the task to the path.
    vector<int> new_path;
    double new_cost = insertTask( robot_num, unalloc[i], new_path );
    // Bid the new path cost.
    bids[robot_num][unalloc[i]] = new_cost;
  }
}

// Selects the winning robot and task by the minimum non-negative bid.
void SequentialAuction::selectWinner(int &winning_robot, int &winning_task){
  // Option - winner is the least contested bid.
  if( use_least_contested_bid && num_robots > 1){
    double max_bid_diff, min_bid, min_bid2, bid_diff, bid;
    int temp_winning_robot = -1;
    max_bid_diff = -1;
    for( int j=0; j<num_tasks; j++ ){
      min_bid = -1;
      min_bid2 = -1;
      for( int i=0; i<num_robots; i++ ){
        bid = bids[i][j];
        if( (bid < min_bid || min_bid == -1) && bid >= 0){
          min_bid2 = min_bid;
          min_bid = bid;
          temp_winning_robot = i;
          continue;
        }
        if( (bid < min_bid2 || min_bid2 == -1) && bid >= 0){
          min_bid2 = bid;
        }
      }
      bid_diff = min_bid2 - min_bid;
      if( bid_diff > max_bid_diff ) {
        max_bid_diff = bid_diff;
        winning_task = j;
        winning_robot = temp_winning_robot;
      }
    }
    return;
  }
  // Option (default) - winner is the lowest bid.
  double min_bid = -1;
  for( int i=0; i<num_robots; i++ ){
    for( int j=0; j<num_tasks; j++ ){
      double bid = bids[i][j];
      if( (bid < min_bid || min_bid == -1) && bid >= 0 ){
        min_bid = bid;
        winning_robot = i;
        winning_task  = j;
      }
    }
  }
}

// Adds the winning task to the winning robots path, and removes the task from
// the unallocated list. Also sets all bids for the winning task to -1.
void SequentialAuction::processWinner(int winning_robot, int winning_task){
  insertTask( winning_robot, winning_task, paths[winning_robot]);
  for( int i=0; i<num_robots; i++ ){
    bids[i][winning_task] = -1;
  }
  for( int i=0; i<unalloc.size(); i++ ){
    if( unalloc[i] == winning_task ){
      unalloc.erase(unalloc.begin()+i);
      return;
    }
  }
}

// Populates the allocations variable.
void SequentialAuction::formOutput(){
  allocations.clear();
  for( int i=0; i<num_robots; i++ ){
    TaskArray ta;
    vector<int> path = paths[i];
    for( int j=0; j<path.size(); j++ ){
      int id = path[j];
      ta.array.push_back(tasks[id]);
    }
    allocations.push_back(ta);
  }
}

/********************************************
 * (low level).
 ********************************************/

// Calculate the new path if a task were to be inserted into a path. Returns
// the cost of the new path.
double SequentialAuction::insertTask( int robot_num, int unalloc_id, vector<int>& new_path){
  vector<int> path = paths[robot_num];
  double      path_cost;
  vector<int> best_path;
  double      best_path_cost = -1;
  for( int i=0; i<=path.size(); i++ ){
    path.insert( path.begin()+i, unalloc_id );
    //printPath( path );
    path_cost = calculatePathCost( robot_num, path );
    //cout << "cost: " << path_cost << endl;
    if( path_cost < best_path_cost || best_path_cost == -1 ){
      best_path_cost = path_cost;
      best_path = path;
    }    
    path.erase( path.begin()+i );
  }
  
  new_path = best_path;
  return best_path_cost;
}

// Calculates the cost to go from the robots start pose to each task in order,
// then returning home (optionally).
double SequentialAuction::calculatePathCost( int robot_num, vector<int> path ){
  double dist = 0;
  double x_prev, y_prev, x_next, y_next, x_diff, y_diff;
  int task_id;
  x_prev = robot_poses[robot_num].position.x;
  y_prev = robot_poses[robot_num].position.y;
  for( int i=0; i<path.size(); i++ ){
    task_id = path[i];
    x_next = tasks[task_id].pose.position.x;
    y_next = tasks[task_id].pose.position.y;
    x_diff = x_next - x_prev;
    y_diff = y_next - y_prev;
    dist += sqrt( x_diff*x_diff + y_diff*y_diff );
    x_prev = x_next;
    y_prev = y_next;  
  }
  if( return_home ){
    x_next = robot_poses[robot_num].position.x;
    y_next = robot_poses[robot_num].position.y;
    x_diff = x_next - x_prev;
    y_diff = y_next - y_prev;
    dist += sqrt( x_diff*x_diff + y_diff*y_diff );
  }
  return dist;
}


/********************************************
 * Printouts, purely for debugging.
 ********************************************/
void SequentialAuction::printPaths(){
  for( int i=0; i<num_robots; i++ ){
    printPath( paths[i] );
  }
}

void SequentialAuction::printPath( vector<int> path ){
  cout << "[ ";
  for( int i=0; i<path.size(); i++ ){
    cout << path[i] << " ";
  }
  cout << "]" << endl;
}

void SequentialAuction::printBids(){
  for( int i=0; i<num_robots; i++ ){
    for( int j=0; j<num_tasks; j++ ){
      printf("% 6.1f ", bids[i][j]);
    }
    cout << endl;
  }
}



 
