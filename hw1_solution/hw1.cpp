#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4
#define QUESTION_5   5
#define QUESTION_6   6
#define QUESTION_6_BIS   7

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm_controller.urdf";
const string robot_name = "PANDA";

// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY = "sai2::cs225a::panda_robot::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::cs225a::panda_robot::sensors::dq";
// - write
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::cs225a::panda_robot::actuators::fgc";
const string CONTROLLER_RUNING_KEY = "sai2::cs225a::controller_running";

unsigned long long controller_counter = 0;

int main() {

	// start redis client
	auto redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	VectorXd initial_q = robot->_q;
	robot->updateModel();

	// prepare controller
	int dof = robot->dof();
	VectorXd command_torques = VectorXd::Zero(dof);

	VectorXd gravity = VectorXd::Zero(dof);
	VectorXd coriolis = VectorXd::Zero(dof);

	int controller_number = QUESTION_6_BIS; 
	string filename;
	if(controller_number == QUESTION_1)
		filename = "../../hw1_solution/data_files/question1.txt";
	else if(controller_number == QUESTION_2)
		filename = "../../hw1_solution/data_files/question2.txt";
	else if(controller_number == QUESTION_3)
		filename = "../../hw1_solution/data_files/question3.txt";
	else if(controller_number == QUESTION_4)
		filename = "../../hw1_solution/data_files/question4.txt";
	else if(controller_number == QUESTION_5)
		filename = "../../hw1_solution/data_files/question5.txt";
	else if(controller_number == QUESTION_6)
		filename = "../../hw1_solution/data_files/question6.txt";
	else if(controller_number == QUESTION_6_BIS)
		filename = "../../hw1_solution/data_files/question6_bis.txt";
	
	ofstream data_file;
	data_file.open(filename);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000); 
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	redis_client.set(CONTROLLER_RUNING_KEY, "1");
	while (runloop) {
		// wait for next scheduled loop
		timer.waitForNextLoop();
		double time = timer.elapsedTime() - start_time;

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		robot->updateModel();

		// **********************
		// WRITE YOUR CODE AFTER
		// **********************
		// int controller_number = QUESTION_1;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 52.0;      // chose your d gain

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired(0) = M_PI/2;

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			if(robot->_q(0) > 1.001*q_desired(0))
			{
				cout << "overshoot" << endl;
			}

			command_torques = - kp * (robot->_q - q_desired) - kv * robot->_dq;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			double kp = 400.0;      
			double kv = 52.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = M_PI/2;

			robot->gravityVector(gravity);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = - kp * (robot->_q - q_desired) - kv * robot->_dq + gravity;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 400.0;      
			double kv = 40.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = M_PI/2;

			robot->gravityVector(gravity);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = robot->_M * (- kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{
			double kp = 400.0;      
			double kv = 40.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = M_PI/2;

			robot->gravityVector(gravity);
			robot->coriolisForce(coriolis);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = robot->_M * (- kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity + coriolis;
		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{
			double kp = 400.0;      
			double kv = 40.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = M_PI/2;

			robot->gravityVector(gravity);
			robot->coriolisForce(coriolis);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = robot->_M * (- kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity + coriolis;
		}

		// ---------------------------  question 6 ---------------------------------------
		if(controller_number == QUESTION_6)
		{
			double kp = 400.0;      
			double kv = 40.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = M_PI/2;

			// compute added mass and gravity
			double mass = 2.5;
			Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.17);
			MatrixXd Jv = MatrixXd::Zero(3,dof);
			robot->Jv(Jv, "link7", pos_in_link);

			VectorXd added_gravity = VectorXd::Zero(dof);
			added_gravity = mass * Jv.transpose() * Vector3d(0,0,9.81);

			MatrixXd added_mass_matrix = MatrixXd::Zero(dof,dof);
			added_mass_matrix = mass * Jv.transpose() * Jv;

			robot->gravityVector(gravity);
			robot->coriolisForce(coriolis);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = (robot->_M + added_mass_matrix) * (- kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity + added_gravity + coriolis;
		}

		// ---------------------------  question 6 ---------------------------------------
		if(controller_number == QUESTION_6_BIS)
		{
			double kp = 400.0;      
			double kv = 40.0;     

			VectorXd q_desired = initial_q; 
			q_desired(0) = initial_q(0) + (M_PI/2 - initial_q(0))*0.0005*controller_counter;
			if(q_desired(0) > M_PI/2)
			{
				q_desired(0) = M_PI/2;
			}

			// compute added mass and gravity
			double mass = 2.5;
			Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.17);
			MatrixXd Jv = MatrixXd::Zero(3,dof);
			robot->Jv(Jv, "link7", pos_in_link);

			VectorXd added_gravity = VectorXd::Zero(dof);
			added_gravity = mass * Jv.transpose() * Vector3d(0,0,9.81);

			MatrixXd added_mass_matrix = MatrixXd::Zero(dof,dof);
			added_mass_matrix = mass * Jv.transpose() * Jv;

			robot->gravityVector(gravity);
			robot->coriolisForce(coriolis);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(0) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t';
				data_file << q_desired(0) << '\t' << q_desired(2) << '\t' << q_desired(3) << '\n';
			}

			command_torques = (robot->_M + added_mass_matrix) * (- kp * (robot->_q - q_desired) - kv * robot->_dq) + gravity + added_gravity + coriolis;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

	data_file.close();


	command_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);
	redis_client.set(CONTROLLER_RUNING_KEY, "0");

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Controller Loop run time  : " << end_time << " seconds\n";
    std::cout << "Controller Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Controller Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


	return 0;
}
