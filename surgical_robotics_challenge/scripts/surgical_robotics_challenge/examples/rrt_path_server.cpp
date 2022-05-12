#include "rrt_path_server.h"
#include <moveit_msgs/RobotTrajectory.h>
#include <moveit/planning_scene/planning_scene.h>
#include <cstdlib>
#include <math.h>
#include "ros/ros.h"

// utility function to test for a collision
bool CS3891Context::is_colliding( const vertex& q ) const {
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q );

  if( getPlanningScene()->isStateColliding( robotstate, "manipulator", false ) )
    { return true; }
  else
    { return false; }
} 

// utility function between two configurations
CS3891Context::vertex CS3891Context::interpolate( const CS3891Context::vertex& qA,
      const CS3891Context::vertex& qB,
      double t ){
  CS3891Context::vertex qt( qA.size(), 0.0 );
  for( std::size_t i=0; i<qt.size(); i++ )
    { qt[i] = ( 1.0 - t )*qA[i] + t*qB[i]; }
  return qt;
}

CS3891Context::CS3891Context( const robot_model::RobotModelConstPtr& robotmodel,
       const std::string& name, 
       const std::string& group, 
       const ros::NodeHandle& nh ) :
  planning_interface::PlanningContext( name, group ),
  robotmodel( robotmodel ){}

CS3891Context::~CS3891Context(){}


// TODO
CS3891Context::vertex CS3891Context::random_sample( const CS3891Context::vertex& q_goal ) const {  
  CS3891Context::vertex q_rand(q_goal.size(),0.0);

  // TODO return a random sample q_rand with goal bias.
  
  do {
   
     double ran = rand();
 
     bool random = fmod(ran, 100) < 95;
 
     if (random){
 	for (int i = 0; i < 6; ++i){
       	    q_rand[i] = (M_PI/180 * (fmod(ran, 360) - 180));
        }
     } else {
	for (int i = 0; i < 6; ++i){
       	    q_rand[i] = q_goal[i];
        }
     }
    
  
  } while (is_colliding(q_rand));
  
  return q_rand;
}

// TODO
double CS3891Context::distance( const CS3891Context::vertex& q1, const CS3891Context::vertex& q2 ){
  double d=0;

  // TODO compute a distance between two configurations (your choice of metric).
  double distance=0;
  for(int i = 0; i < q1.size(); ++i){
    if (i >= 3 & abs(q1[i] - q2[i]) > M_PI) {
	distance += pow((2*M_PI - abs(q1[i] - q2[i])), 2);
    } else {
	distance += pow(abs(q1[i] - q2[i]),2);
    }
  }

  d = pow(distance, 0.5);
  
  return d;
}

// TODO
CS3891Context::vertex CS3891Context::nearest_configuration( const CS3891Context::vertex& q_rand ){
  CS3891Context::vertex q_near(q_rand.size(),0.0);

  // TODO find the nearest configuration in the tree to q_rand
  double nearest = FLT_MAX;

  for(int i = 0;i < treeVertex.size(); ++i){
    CS3891Context::vertex vertex = treeVertex[i];

    double dist = distance(vertex, q_rand);

    if (dist < nearest){
      nearest = dist;
      q_near = vertex;
    }
  }
  
  return q_near;
}

// TODO
bool CS3891Context::is_subpath_collision_free( const CS3891Context::vertex& q_near,
           const CS3891Context::vertex& q_rand ){
  
  // TODO find if the straightline path between q_near and q_rand is collision free
  // 100 is the max iteration
  CS3891Context::vertex q_temp = q_near;
  for(int i = 0; i < 100; ++i){
      q_temp = interpolate(q_temp, q_rand, ((double)i)/100.0);

      if(is_colliding(q_temp)){
	  std::cout << "collision found!!" << std::endl;
          return false;
      }
  }

  return true;
}


// TODO
CS3891Context::path CS3891Context::search_path( const CS3891Context::path& P, 
	   const CS3891Context::vertex& q_init,
           const CS3891Context::vertex& q_goal ){

  // TODO Once q_goal has been added to the tree, find the path (sequence of configurations) between
  // q_init and q_goal (hint: this is easier by using recursion).
  CS3891Context::path newPath = P;
  CS3891Context::vertex q_cur = q_goal;
  newPath.push_back(q_cur);
  if(q_cur == q_init){
    return newPath;
  }
  for(std::pair<vertex,vertex> edge : treeEdge){
      if (edge.second == q_cur){
        return search_path(newPath, q_init, edge.first);
      }
  }
}

// TODO
CS3891Context::path CS3891Context::rrt( const CS3891Context::vertex& q_init,
          const CS3891Context::vertex& q_goal ){
  CS3891Context::path P;

  // TODO implement RRT algorithm and return the path (an ordered sequence of configurations).
  // initialize tree
  treeVertex.push_back(q_init);

  CS3891Context::vertex q_rand;
  CS3891Context::vertex q_near;
  CS3891Context::vertex q_new;
  while (std::find(treeVertex.begin(), treeVertex.end(), q_goal) == treeVertex.end()){
    q_rand = random_sample(q_goal);
    q_near = nearest_configuration(q_rand);

    q_new = interpolate(q_near, q_rand, 0.5);
    if (is_subpath_collision_free(q_near, q_new)){
        // add config q_new and edge(q_near, q_new) to tree
	std::cout << "treevertex" << treeVertex.size() << std::endl;
        treeVertex.push_back(q_new);
        treeEdge.push_back(std::make_pair(q_near, q_new));
	if (distance(q_new, q_goal) < 10){
	  treeVertex.push_back(q_goal);
	  treeEdge.push_back(std::make_pair(q_new, q_goal));
	  CS3891Context::path P;
          P = search_path(P, q_init, q_goal);
	  std::cout << "found" << std::endl;
	  for (int i = 0; i < P.size(); i++) {
	    std::cout << P[i][0] << "+" << P[i][1] << "+" << P[i][2] << "+" << P[i][3] << "+" << P[i][4] << "+" << P[i][5] << std::endl;
	  }
          return P;
        }
    }
  }
  
  return P;
}

// This is the method that is called each time a plan is requested
bool CS3891Context::solve( planning_interface::MotionPlanResponse &res ){
  std::cout << "The begin of solve()." << std::endl;
  // Create a new empty trajectory
  res.trajectory_.reset(new robot_trajectory::RobotTrajectory(robotmodel, getGroupName()));
  res.trajectory_->clear();

  treeVertex.clear();
  treeEdge.clear();

  // copy the initial/final joints configurations to vectors qfin and qini
  // This is mainly for convenience.
  vertex q_init, q_goal;
  for( size_t i=0; i<robotmodel->getVariableCount(); i++ ){
    q_goal.push_back(request_.goal_constraints[0].joint_constraints[i].position);
    q_init.push_back(request_.start_state.joint_state.position[i]);
  }

  // start the timer
  ros::Time begin = ros::Time::now();

  path P = rrt( q_init, q_goal );

  // end the timer
  ros::Time end = ros::Time::now();

  // The rest is to fill in the animation. You can ignore this part.
  moveit::core::RobotState robotstate( robotmodel );
  robotstate.setJointGroupPositions( "manipulator", q_init );

  for( std::size_t i=P.size()-1; i>=1; i-- ){
    for( double t=0.0; t<=1.0; t+=0.01 ){
      vertex q = interpolate( P[i], P[i-1], t );
      robotstate.setJointGroupPositions( "manipulator", q );
      res.trajectory_->addSuffixWayPoint( robotstate, 0.01 );
    }
  }

  // set the planning time
  ros::Duration duration = end-begin;
  res.planning_time_ = duration.toSec();
  res.error_code_.val = moveit_msgs::MoveItErrorCodes::SUCCESS;

  std::cout << "The end of solve()." << std::endl;
  return true;
  
}

bool CS3891Context::solve( planning_interface::MotionPlanDetailedResponse &res )
{ std::cout << "outputtttt" << std::endl;
return true; }

void CS3891Context::clear(){}

bool CS3891Context::terminate(){return true;}

bool greet(project::::Request  &req,
		beginner_tutorials::AddTwoInts::Response &res){return true;}

int main(int argc, char **argv)
{
  std::cout << "outputtttt" << std::endl;
  ros::init(argc, argv, "rrt_path_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("solve_rrt", greet);
  ROS_INFO("Ready to solve for rrt path.");
  
  ros::spin();

  return 0;
}
