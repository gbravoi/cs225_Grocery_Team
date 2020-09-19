#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;

// Location of URDF files specifying world and robot information
const string world_file = "./resources/world.urdf";
const string robot_file = "./resources/rprbot.urdf";
const string robot_name = "RPRBot";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - write:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::RPRbot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::RPRbot::sensors::dq";

int main() {
	cout << "Loading URDF world model file: " << world_file << endl;

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	RedisClient redis_client = RedisClient();
	redis_client.connect();

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, true);

	/*
	These are mathematical vectors from the library Eigen, you can read up on the documentation online.
	You can input your joint information and read sensor data C++ style "<<" or ">>". Make sure you only 
	expect to read or are writing #D.O.F. number of values.
	*/
	robot->_q << 0, 0.6, M_PI/3; // Joint 1,2,3 Coordinates (radians, meters, radians)
	robot->_dq << 0, 0, 0; // Joint 1,2,3 Velocities (radians/sec, meters/sec, radians/sec), not used here

	/* 
	Here we use our redis set method to serialize an 'Eigen' vector into a specific Redis Key
	Changing set to get populates the 'Eigen' vector given
	This key is then read by the physics integrator or visualizer to update the system
	*/
	redis_client.setEigenMatrixJSON(JOINT_ANGLES_KEY,robot->_q);
	redis_client.setEigenMatrixJSON(JOINT_VELOCITIES_KEY, robot->_dq);

	/*
	Update model calculates and updates robot kinematics model information 
	(calculate current jacobian, mass matrix, etc..)
	Values taken from robot-> will be updated to currently set _q values
	*/
	robot->updateModel();

	int dof = robot->dof();
	cout << endl << endl;

	// operational space
	std::string ee_link_name = "link2"; // Link of the "Task" or "End Effector"

	// Position of Task Frame in relation to Link Frame (When using custom E.E. attachment, etc..)
	// ---------------------------------------------------------------------------------------
	//    -----------------    YOU WILL NEED TO CHANGE THIS ONE  -----------------------------
	// ---------------------------------------------------------------------------------------
	Eigen::Vector3d ee_pos_in_link = Eigen::Vector3d(0.0, 0.0, 0.0); 
	
	Eigen::Vector3d ee_position = Eigen::Vector3d::Zero(); // 3d vector of zeros to fill with the end effector position
	Eigen::MatrixXd ee_jacobian(3,dof); // Empty Jacobian Matrix sized to right size
	Eigen::VectorXd g(dof); // Empty Gravity Vector

	robot->position(ee_position, ee_link_name, ee_pos_in_link);
	cout << "end effector position (remember that 0,0,0 is the position of the last revolute joint, not the end effector)" << endl;
	cout << ee_position.transpose() << endl;

	robot->Jv(ee_jacobian,ee_link_name,ee_pos_in_link); // Read jacobian into ee_jacobian
	cout << "printing Jacobian and Mass matrix : " << endl;
	cout << ee_jacobian << endl; // Print Jacobian
	cout << robot->_M << endl; // Print Mass Matrix, you can index into this variable (and all 'Eigen' types)!

	robot->gravityVector(g); // Fill in and print gravity vectory
	cout << "printing gravity : " << endl;
	cout << endl << g.transpose() << endl;

	/* 
	Retrieve multiple values of the gravity or M with a for loop of setting robot->_q's, 
	setting redis keys for display update if needed and don't forget robot->updateModel()! 
	We'll have a logger for you later to dump redis values at whatever rate you choose
	*/

    return 0;
}
