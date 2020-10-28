#include <ros/ros.h>
#include "PathPlan/PathPlan.hpp"
#include "PreDefine.hpp"
#include <math.h>
#include <algorithm>

/*
 * Constructor
 */

PathPlan::PathPlan(ros::NodeHandle& nh)
{
  nh_ = nh;
  range_sub_ = nh_.subscribe("/range_pub", 1, &PathPlan::rangeCallback, this);
  odom_sub_ = nh_.subscribe("/odom", 1, &PathPlan::odomCallback, this);
  target_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/target", 1);
  curr_pub_ = nh_.advertise<geometry_msgs::Point>("/pathplan/curr", 1);
  
  target_x_ = 0;
  target_y_ = 0;
  target_x_prev_ = 0;
  target_y_prev_ = 0;
  
  goal_reached_ = GOAL_NOT_REACH;
  
  initializeWall();
  initializePathMap();

  // add the starting point to history
  history.push(std::pair<int, int>(0, 0));

  ROS_INFO("PathPlan node initialized successfully!");
  
}

void PathPlan::spin()
{
  ros::Rate loop_rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
  }
}

/*
 * Callback Function
 */

void PathPlan::odomCallback(const nav_msgs::OdometryConstPtr& odomMsg)
{
  //use the first pose of Turtlebot as the original point, with front pointing to y axis
  pos_y_ = odomMsg->pose.pose.position.y;
  pos_x_ = odomMsg->pose.pose.position.x;
  
  ROS_INFO("Received /odom position (%f, %f).", pos_x_, pos_y_);
  
  float q_x = odomMsg->pose.pose.orientation.x;
  float q_y = odomMsg->pose.pose.orientation.y;
  float q_z = odomMsg->pose.pose.orientation.z;
  float q_w = odomMsg->pose.pose.orientation.w;
  ang_z_ = atan2(2*(q_w*q_z + q_x*q_y), 1-2*(q_z*q_z + q_y*q_y)); //because in the world frame, there is a 90 degree transformation
  //mind that ang_z_ is with respect to the x axis of the world frame
  
  // check the wall distribution and update the path map every time the robot localize itself
  checkWall();
  //wall_map.print();
  
  path_plan_alg();
  if(!goal_reached_){
    setNextDestCell();
  }
  
  geometry_msgs::Point target_point;
  geometry_msgs::Point curr_position;
  
  target_point.x = target_x_;
  target_point.y = target_y_;
  target_point.z = goal_reached_;
  
  curr_position.x = pos_x_;
  curr_position.y = pos_y_;
  curr_position.z = ang_z_*180/PI; //in degree
  
  ROS_INFO("target: X: %d, Y: %d", target_x_, target_y_);
  ROS_INFO("ang_z_: %f", ang_z_ /PI*180);
  
  target_pub_.publish<geometry_msgs::Point>(target_point);
  curr_pub_.publish<geometry_msgs::Point>(curr_position);
  
}

void PathPlan::rangeCallback(const std_msgs::Float32MultiArray& rangeMsg)
{
  dist_north_ = rangeMsg.data[0];
  dist_east_ = rangeMsg.data[1];
  dist_south_ = rangeMsg.data[2];
  dist_west_ = rangeMsg.data[3];
  
}


/*
 * Wall Map related
 */
void PathPlan::checkWall()
{
  //in world frame
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);
  
  if(dist_north_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, NORTH);
  } 
  if(dist_north_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, NORTH);
  }
  
  if(dist_east_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, EAST);
  } 
  if(dist_east_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, EAST);
  }
  
  if(dist_south_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, SOUTH);
  } 
  if(dist_south_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, SOUTH);
  }
  
  if(dist_west_ < WALL_DETECT_DIST){
    setWall(pos_x_int, pos_y_int, WEST);
  } 
  if(dist_west_ > OPEN_DETECT_DIST){
    removeWall(pos_x_int, pos_y_int, WEST);
  }
  
}


void PathPlan::setWall(int x, int y, int direction)
{
  if(x<0 || y <0 || direction <0){
    ROS_ERROR("setWall function input with illegal: x:%d, y:%d, direction:%d", x, y, direction);
    return;
  }
  wall_map(x, y, direction) = WALL;
  ROS_INFO("Wall in %d", direction);
  
  // fill wall to adjacent cells
  if(direction == NORTH){
    if(y+1 < GRID_SIZE) wall_map(x, y+1, SOUTH) = WALL;
    return;
  }
  if(direction == SOUTH){
    if(y-1 >= 0) wall_map(x, y-1, NORTH) = WALL;
    return;
  }
  if(direction == WEST){
    if(x-1 >= 0) wall_map(x-1, y, EAST) = WALL;
    return;
  }
  if(direction == EAST){
    if(x+1 < GRID_SIZE) wall_map(x+1, y, WEST) = WALL;
    return;
  }
  
}


bool PathPlan::hasWall(int x, int y, int direction)
{
  if(x<0 || y<0 || direction<0){
    ROS_ERROR("hasWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
    return true; //return with positive wall
  }
  if(wall_map(x, y, direction) == WALL) {
    //ROS_INFO("Cell(%d, %d, %d) has Wall in the map!", x, y, direction);
    return true;
  }
  else return false;
}

void PathPlan::removeWall(int x, int y, int direction)
{
  if(x<0 || y<0 || direction<0){
    ROS_ERROR("removeWall get an illegal input: x:%d, y:%d, direction:%d", x, y, direction);
    return;
  }
  wall_map(x, y, direction) = OPEN;
  //ROS_INFO("Remove Wall for Cell(%d, %d, %d).", x, y, direction);
}


void PathPlan::initializeWall()
{
  //mind that the map we draw has a different orientation with the cell matrix 
  wall_map = cube(GRID_SIZE, GRID_SIZE, 4, fill::zeros); //each cell has four direction, with 0 on one direction means open on that direction
  
  for(int row = 0; row < GRID_SIZE; row++){
    for(int col = 0; col < GRID_SIZE; col++){
      if(row == 0){
        wall_map(row, col, WEST) = WALL;
      }
      if(row == GRID_SIZE-1){
        wall_map(row, col, EAST) = WALL;
      }
      if(col == 0){
        wall_map(row, col, SOUTH) = WALL;
      }
      if(col == GRID_SIZE-1){
        wall_map(row, col, NORTH) = WALL;
      }
    }
  }
}

/*
 * path map related
 */
void PathPlan::initializePathMap()
{
  path_map_ = Mat<int>(GRID_SIZE, GRID_SIZE); //set all the path value as inf
  path_map_.fill(1000);
  path_map_(GOAL_X, GOAL_Y) = 0;
  path_map_initialized_ = true;
  //update the path plan with current obstacles
  //dijkstra();
}

void PathPlan::path_plan_alg()
{
  // make sure the goal cell has been put in the path_map
  if(!path_map_initialized_) initializePathMap();
  
  mat visited_map(GRID_SIZE, GRID_SIZE, fill::zeros);
  mat is_queued(GRID_SIZE, GRID_SIZE, fill::zeros);
  
  std::vector<pair<int, int>> reached_queue;
  reached_queue.push_back(pair<int, int>(GOAL_X, GOAL_Y));
  
  int nVisited = 0;
  int nCells = GRID_SIZE * GRID_SIZE;
  int mdist_from_goal_node = 0;
  pair<int, int> node;
  
  int x_queue, y_queue; //tmp coord in queue
  
  while(nVisited < nCells){
    int min_dist = 1000;
    int queue_length = reached_queue.size();
    //ROS_INFO("queue length: %d", queue_length);
    
    for(int i = 0; i<queue_length; i++){
      pair<int, int> tmp = reached_queue[i];
      //find the minimum goal distance node that hasn't been visited

      if(visited_map.at(tmp.first, tmp.second) == 0){
      	if(path_map_(tmp.first, tmp.second) < min_dist){
      		min_dist = path_map_.at(tmp.first, tmp.second);
      		node = tmp;
      	}
      }
    }
    
    visited_map(node.first, node.second) = 1; //this node has been visited
    //visited_map.print();
    nVisited++;
    //ROS_INFO("Add %d nodes. min_dist: %d", nVisited, min_dist);
    
    // extend to adjacent cells to this node
    for(int direction = NORTH; direction <= WEST; direction++){
      
      if(!hasWall(node.first, node.second, direction)){
        // there is no wall on that direction, and the adjacent node hasn't been put in the queue_length
        
        if(direction == NORTH && node.second+1 < GRID_SIZE){
          x_queue = node.first; y_queue = node.second+1;
        }
        if(direction == EAST && node.first+1 < GRID_SIZE){
          x_queue = node.first+1; y_queue = node.second;
        }
        if(direction == SOUTH && node.second-1 >= 0){
          x_queue = node.first; y_queue = node.second-1;
        }
        if(direction == WEST && node.first-1 >= 0){
          x_queue = node.first-1; y_queue = node.second;
        }
        
        if(!is_queued(x_queue, y_queue)){
          // put the adjacent cell in the queue
          reached_queue.push_back(pair<int, int>(x_queue, y_queue));
          is_queued(x_queue, y_queue) = 1;
        }
        
        // update the path_map, update the shortest path to goal
        path_map_(x_queue, y_queue) = min(path_map_(x_queue, y_queue), min_dist+1);
        
        //ROS_INFO("%d no wall. (%d, %d) path_map is %d", direction, x_queue, y_queue, path_map(x_queue, y_queue));
      }
	
    }
    //test print the path_map
    // path_map_.print();    
  }

  
}


void PathPlan::setNextDestCell()
{
  // get current location of robot
  int pos_x_int = (int)floor(pos_x_);
  int pos_y_int = (int)floor(pos_y_);

  ROS_INFO("pos_x_int:%d, pos_y_int:%d", pos_x_int, pos_y_int);

  // stop if goal is reached
  if(pos_x_int == GOAL_X && pos_y_int == GOAL_Y){
    goal_reached_ = GOAL_REACH;
    ROS_INFO("Goal Reach!, X:%d, Y:%d", GOAL_X, GOAL_Y);
    return;
  }

  ROS_INFO("history size: %d", (int)history.size());

  pair<int, int> node = history.top();

  // return if robot has not reached target goal
  if (!(abs(pos_x_ - (node.first + 0.5)) <= 0.1 && abs(pos_y_ - (node.second + 0.5)) <= 0.1)) {
    return;
  }


  if (explored.find(node) == explored.end()) {
    ROS_INFO("Moving to unvisited node");
    // for nodes that are unvisited take the north direction first
    int x, y;
    int heading = UNDEFINED;
    bool is_all_blocked = true;
    for(int direction = NORTH; direction <= WEST; direction++){
      if(!hasWall(pos_x_int, pos_y_int, direction)) {
        is_all_blocked = false;
        int tmp_x, tmp_y;
        if(direction == NORTH) {
          tmp_x = pos_x_int; tmp_y = pos_y_int + 1;
        } else if (direction == EAST) {
          tmp_x = pos_x_int + 1; tmp_y = pos_y_int;
        } else if (direction == SOUTH) {
          tmp_x = pos_x_int; tmp_y = pos_y_int - 1; 
        } else if (direction == WEST) {
          tmp_x = pos_x_int -1; tmp_y = pos_y_int;
        } else {
          return;
        }

        // continue if the next node is explored
        if (explored.find(pair<int, int>(tmp_x, tmp_y)) != explored.end()) {
          continue;
        }
        heading = direction;
        x = tmp_x;
        y = tmp_y;
        break;
      }
    }

    // return if all walls are blocked at the starting position
    if (is_all_blocked) {
      return;
    }

    // set direction is 5 when all directions for that node has been explored to force bactracking
    if (heading == UNDEFINED) {
      explored[node] = 5;
      return;
    }

    explored[node] = heading;
    history.push(pair<int, int>(x, y));
    target_x_prev_ = target_x_;
    target_y_prev_ = target_y_;
    target_x_ = x;
    target_y_ = y;
  } else {
    // backtrack or visit neighbouring nodes if nodes have been explored
    int heading = UNDEFINED;

    // check for unexplored neighbours
    for(int direction = explored[node] + 1; direction <= WEST; direction++){
      if(!hasWall(pos_x_int, pos_y_int, direction)) {
        int tmp_x, tmp_y;
        if(direction == NORTH) {
          tmp_x = pos_x_int; tmp_y = pos_y_int + 1;
        } else if (direction == EAST) {
          tmp_x = pos_x_int + 1; tmp_y = pos_y_int;
        } else if (direction == SOUTH) {
          tmp_x = pos_x_int; tmp_y = pos_y_int - 1; 
        } else if (direction == WEST) {
          tmp_x = pos_x_int -1; tmp_y = pos_y_int;
        } else {
          return;
        }
        if (explored.find(pair<int, int>(tmp_x, tmp_y)) != explored.end()) {
          ROS_INFO("node %d, %d has been visited", tmp_x, tmp_y);
          continue;
        }
        heading = direction;
        break;
      }
    }
    ROS_INFO("Heading: %d", heading);

    if (heading == UNDEFINED) {
      // backtracks when all nodes have been explored and goal is not reached
      ROS_INFO("Backtracing");
      history.pop();
      pair<int, int> prev = history.top();
      int x = prev.first;
      int y = prev.second;
      target_x_prev_ = x;
      target_y_prev_ = y;
      target_x_ = x;
      target_y_ = y;
      ROS_INFO("Backtracing to x, y %d %d", x, y);
    } else {
      // visit neighbouring node
      ROS_INFO("Moving to next neighbouring node");
      int x, y;

      if (heading == NORTH) {
        x = pos_x_int; y = pos_y_int+1;
      }
      if (heading == EAST) {
        x = pos_x_int+1; y = pos_y_int;
      }
      if (heading == SOUTH) {
        x = pos_x_int; y = pos_y_int-1;
      }
      if (heading == WEST) {
        x = pos_x_int-1; y = pos_y_int;
      }

      explored[node] = heading;
      history.push(pair<int, int>(x, y));
      target_x_prev_ = target_x_;
      target_y_prev_ = target_y_;
      target_x_ = x;
      target_y_ = y;
    }
  }
}

int PathPlan::getPathMapValue(int x, int y)
{
  if(x<0 || x>=GRID_SIZE || y<0 || y>=GRID_SIZE){
    ROS_ERROR("get_path_map_value get an illegal input x:%d, y%d", x, y);
    return 1000;
  }
  if(!path_map_initialized_){
    ROS_ERROR("path_map hasn't initialized.");
    return 1000;
  }
  
  return path_map_.at(x, y);
  
}

