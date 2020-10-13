#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>
#include <fstream>

#define QUESTION_1		1
#define QUESTION_2_a	2
#define QUESTION_2_c	3
#define QUESTION_2_d	4
#define QUESTION_3		5
#define QUESTION_4_i 	6
#define QUESTION_4_ii   7
#define QUESTION_4_iii  8
#define QUESTION_4_iv   9

// handle ctrl-c nicely
#include <signal.h>
bool runloop = true;
void sighandler(int sig)
{ runloop = false; }

using namespace std;
using namespace Eigen;

const string robot_file = "./resources/panda_arm.urdf";
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
	const string link_name = "link7";
	const Vector3d pos_in_link = Vector3d(0, 0, 0.1);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);
	VectorXd g = VectorXd::Zero(dof);
	VectorXd b = VectorXd::Zero(dof);
	Vector3d x;
	Vector3d v;
	VectorXd F(dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	int controller_number = QUESTION_4_iv; 
	string filename;
	if(controller_number == QUESTION_1)
		filename = "../../hw2_solution/data_files/question1.txt";
	else if(controller_number == QUESTION_2_a)
		filename = "../../hw2_solution/data_files/question2a.txt";
	else if(controller_number == QUESTION_2_c)
		filename = "../../hw2_solution/data_files/question2c.txt";
	else if(controller_number == QUESTION_2_d)
		filename = "../../hw2_solution/data_files/question2d.txt";
	else if(controller_number == QUESTION_3)
		filename = "../../hw2_solution/data_files/question3.txt";
	else if(controller_number == QUESTION_4_i)
		filename = "../../hw2_solution/data_files/question4i.txt";
	else if(controller_number == QUESTION_4_ii)
		filename = "../../hw2_solution/data_files/question4ii.txt";
	else if(controller_number == QUESTION_4_iii)
		filename = "../../hw2_solution/data_files/question4iii.txt";
	else if(controller_number == QUESTION_4_iv)
		filename = "../../hw2_solution/data_files/question4iv.txt";
	
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

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			auto q_desired = initial_q;
			q_desired(6) = 0.1;
			DiagonalMatrix<double, 7> Kp(7);
			Kp.diagonal() << 400,400,400,400,400,400,50;
			DiagonalMatrix<double, 7> Kv(7);
			Kv.diagonal() << 50,50,50,50,50,50,-0.34;
			// robot->coriolisPlusGravity(g);
			robot->gravityVector(g);
			robot->coriolisForce(b);

			if(controller_counter % 10 == 0)
			{
				data_file << robot->_q(6) << '\t' << q_desired(6) << '\n';
			}

			command_torques = Kp * (q_desired - robot->_q) - Kv * (robot->_dq) + b + g;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2_a)
		{
			double kp = 200;
			double kv = 23.5;
			double kvj = 5;
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);
			Vector3d xd(0.3, 0.1, 0.5);
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}

			F = Lambda * (kp * (xd - x) - kv * v);
 			command_torques = Jv.transpose() * F + g - 0 * N.transpose() * robot->_M * kvj * robot->_dq - 0 * kvj * robot->_dq;
 		}

 		if(controller_number == QUESTION_2_c)
		{
			double kp = 200;
			double kv = 23.5;
			double kvj = 5;
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);
			Vector3d xd(0.3, 0.1, 0.5);
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}

			F = Lambda * (kp * (xd - x) - kv * v);
			command_torques = Jv.transpose() * F + g - 0 * N.transpose() * robot->_M * kvj * robot->_dq - kvj * robot->_dq;
		}

 		if(controller_number == QUESTION_2_d)
		{
			double kp = 200;
			double kv = 23.5;
			double kvj = 5;
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);
			Vector3d xd(0.3, 0.1, 0.5);
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}

			F = Lambda * (kp * (xd - x) - kv * v);
			command_torques = Jv.transpose() * F + g - N.transpose() * robot->_M * kvj * robot->_dq - 0 * kvj * robot->_dq;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			double kp = 200;
			double kv = 23.5;
			double kvj = 5;
			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);
			Vector3d xd(0.3, 0.1, 0.5);
			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}
			
			F = Lambda * (kp * (xd - x) - kv * v) + J_bar.transpose() * g;
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * kvj * robot->_dq;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4_i)
		{
			double kp = 200;
			double kv = 24;
			double kpj = 25;
			double kvj = 10;

			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);

			Vector3d xd(0.3, 0.1, 0.5);
			xd(0) += 0.1 * sin(M_PI * time);
			xd(1) += 0.1 * cos(M_PI * time);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			auto q = robot->_q;

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}
			
			F = Lambda * (kp * (xd - x) - kv * v) + J_bar.transpose() * g;
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * kvj * robot->_dq;
		}

		if(controller_number == QUESTION_4_ii)
		{
			double kp = 200;
			double kv = 24;
			double kpj = 25;
			double kvj = 10;

			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);

			Vector3d xd(0.3, 0.1, 0.5);
			xd(0) += 0.1 * sin(M_PI * time);
			xd(1) += 0.1 * cos(M_PI * time);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			auto q = robot->_q;

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}
			
			F = (kp * (xd - x) - kv * v) + J_bar.transpose() * g;
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * kvj * robot->_dq;
		}

		if(controller_number == QUESTION_4_iii)
		{
			double kp = 200;
			double kv = 24;
			double kpj = 25;
			double kvj = 10;

			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);

			Vector3d xd(0.3, 0.1, 0.5);
			xd(0) += 0.1 * sin(M_PI * time);
			xd(1) += 0.1 * cos(M_PI * time);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			auto q = robot->_q;

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}
			
			F = Lambda * (kp * (xd - x) - kv * v) + J_bar.transpose() * g;
			command_torques = Jv.transpose() * F - N.transpose() * robot->_M * (kvj * robot->_dq + kpj * q);
		}

		if(controller_number == QUESTION_4_iv)
		{
			double kp = 200;
			double kv = 24;
			double kpj = 25;
			double kvj = 10;

			robot->position(x, link_name, pos_in_link);
			robot->linearVelocity(v, link_name, pos_in_link);

			Vector3d xd(0.3, 0.1, 0.5);
			xd(0) += 0.1 * sin(M_PI * time);
			xd(1) += 0.1 * cos(M_PI * time);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->gravityVector(g);
			robot->nullspaceMatrix(N,Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			auto q = robot->_q;

			if(controller_counter % 10 == 0)
			{
				data_file << x(0) << '\t' << x(1) << '\t' << x(2) << '\t';
				data_file << xd(0) << '\t' << xd(1) << '\t' << xd(2) << '\t';
				data_file << robot->_q(0) << '\t' << robot->_q(1) << '\t' << robot->_q(2) << '\t' << robot->_q(3) << '\t' << robot->_q(4) << '\t' << robot->_q(5) << '\t' << robot->_q(6) << '\n';
			}
			
			F = Lambda * (kp * (xd - x) - kv * v) + J_bar.transpose() * g;
			command_torques = Jv.transpose() * F - N.transpose() * (robot->_M * (kvj * robot->_dq + kpj * q) - g);
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;
		if (controller_counter == 10000) runloop = false;

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
