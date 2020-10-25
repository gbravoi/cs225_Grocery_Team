#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"

#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

#include <signal.h>
bool runloop = false;
void sighandler(int){runloop = false;}

// helper function
double sat(double x) {
	if (abs(x) <= 1.0) {
		return x;
	}
	else {
		return signbit(x);
	}
}

#define RAD(deg) ((double)(deg) * M_PI / 180.0)

// Location of URDF files specifying world and robot information
const string robot_file = "./resources/mmp_panda.urdf";
const string obj_file = "./resources/cup.urdf";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::panda::sensors::dq";
const std::string OBJ_JOINT_ANGLES_KEY  = "cs225a::object::cup::sensors::q";
const std::string OBJ_JOINT_VELOCITIES_KEY = "cs225a::object::cup::sensors::dq";
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::robot::panda::actuators::fgc";

// manual object offset since the offset in world.urdf file since positionInWorld() doesn't account for this
// Eigen::Vector3d obj_offset;
// obj_offset << 0, -0.35, 0.544;
// Eigen::Vector3d robot_offset;
// robot_offset << 0.0, 0.3, 0.0;


//state machine states
enum Simulation_states {
	IDLE=1,
	GO_TO_SHELF,
	PICK_SHELF_OBJECTS,
	GO_TO_CONVEYOR,
	PLACE_OBJECS_CONVEYOR
	};

enum Robot_States {
	R_IDLE = 1,
	PICKING_OBJECT,
	PLACING_OBJECT,
	MOVING_ARM,
	ORIENTING_ARM,
	NAVIGATING };

enum Gripper_States{
	OPENING_GRIPPER,
	CLOSING_GRIPPER,
};

enum Events {
	CLOSE_TO_OBJECT = 1,
	CONTACT_DETECTED,
	GRIPPER_REACHED_FORCE,
	EE_REACHED_POSITION,
	GRIPPER_OPEN };


class Grocery_Robot{
	VectorXd control_torques;

	public:
		Robot_States Current_state=Robot_States::R_IDLE; //robot state
		//simulation
		Sai2Model::Sai2Model* robot;
		//dynamic properties
		Eigen::MatrixXd J0; //end effector basic Jacobian
		Eigen::MatrixXd L0; //Lambda_0 at end effector
		MatrixXd N0;
		MatrixXd Jv;
		MatrixXd Lambda;
		MatrixXd J_bar;
		MatrixXd N;
		VectorXd g;
		//current states
		Vector3d x;//current position
		Vector3d x_vel;//current velocity
		Vector3d x_acc;//current acceleration
		Vector3d w;//current angular velocity
		VectorXd q;//current joints angle
		VectorXd dq;//current joints velocity
		Eigen::Matrix3d R;//current orientation
		//desired states
		Vector3d x_des;//goal position
		Vector3d x_vel_des;//goal velocity
		Vector3d x_acc_des;//goal acceleration
		VectorXd q_des;//goal joints angle
		Eigen::Matrix3d R_des;//desired orientation
		double q_gripper_goal;//gripper displacement (same for both)
		//time, waypoints
		double cur_time;
		MatrixXd basket_to_shelf_waypoints;
		double last_waypoint_arrival_time;
		double max_time_btw_waypoints = 2.0;
		int waypoint_iterator=-1;
		double distance_thres = 0.01;
		//control constant
		double kp ;      // chose your p gain
		double kv;      // chose your d gain
		double kvj; //joint damping
		double kpj; //joint p gain
		double V_max; //max velocity

		//Robot properties
		int dof;
		const string link_name = "link7";
		const Vector3d pos_in_link = Vector3d(0, 0, 0.15);




		VectorXd Position_controller(bool joint_control);
		void Update_states();
		void set_robot_state(Robot_States robot_state);
		bool checkWaypoints(MatrixXd& waypoints, int num_waypoints);
};

void Grocery_Robot::Update_states(){
	// update robot model and compute gravity, and other stuff
	robot->updateModel();
	robot->gravityVector(g);
	q=robot->_q;
	dq=robot->_dq;
	robot->positionInWorld(x, link_name, pos_in_link);
	robot->linearVelocity(x_vel, link_name, pos_in_link);
	robot->angularVelocity(w, link_name);
	robot->rotation(R, link_name);

	robot->J_0(J0, link_name, pos_in_link);
	robot->nullspaceMatrix(N0, J0);
	robot->taskInertiaMatrix(L0, J0);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
 };

void Grocery_Robot::set_robot_state(Robot_States robot_state){
	Current_state=robot_state;
	cout << "Current Robot State "<<  robot_state <<endl;
};

VectorXd Grocery_Robot::Position_controller(bool joint_control){
		Vector3d delta_phi;
		delta_phi = -0.5 * (R.col(0).cross(R_des.col(0)) + R.col(1).cross(R_des.col(1)) + R.col(2).cross(R_des.col(2)));

		double Vmax = 0.5;
		x_vel_des = - kp / kv * (x - x_des);
		double nu = sat(Vmax / x_vel_des.norm());

		//joint desired state
		if (joint_control==false){
			q_des=robot->_q;//all other joints in current position
			}
		//Change end effector position
		q_des(10)=q_gripper_goal;
		q_des(11)=-1*q_gripper_goal;

		Vector3d pd_x = - kp * nu * (x - x_des) - kv * x_vel;
		Vector3d pd_w = kp * (- delta_phi) - kv * w;
		VectorXd pd(6);
		pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];

		VectorXd F(6);
		F = L0 * pd;
		control_torques = J0.transpose() * F + N.transpose() * ( - (kvj * robot->_dq) -robot->_M*kpj*(robot->_q-q_des)) + 0*g;  // gravity is compensated in simviz loop as of now

		return control_torques;
 };

bool Grocery_Robot::checkWaypoints(MatrixXd& waypoints, int num_waypoints){
	if(waypoint_iterator == -1){
		waypoint_iterator = 0;
		last_waypoint_arrival_time = cur_time;
		x_des = waypoints.col(waypoint_iterator);
	}

	Vector3d dist = x-waypoints.col(waypoint_iterator);
	if(dist.norm() < distance_thres || cur_time - last_waypoint_arrival_time > max_time_btw_waypoints){
		waypoint_iterator++;
		last_waypoint_arrival_time = cur_time;
		if(waypoint_iterator >= num_waypoints){
			waypoint_iterator = -1;
			return true;
		}else{
			x_des = waypoints.col(waypoint_iterator);
		}
	}
	return false;
}

//State machine sub functions
VectorXd pick_shelf_objects(Grocery_Robot *Robot){
	bool arrived = Robot->checkWaypoints(Robot->basket_to_shelf_waypoints, 5);
	VectorXd control_torques= VectorXd::Zero(Robot->dof);
	double control_threshold=0.07;
	control_torques=Robot->Position_controller(false);

	switch (Robot->Current_state)
	{
	case PICKING_OBJECT:
		control_torques.setZero();
		break;
	case PLACING_OBJECT:
		break;
	case MOVING_ARM:
		if( (Robot->x_des-Robot->x).norm()>control_threshold){
			control_torques=Robot->Position_controller(false);
			//cout<<(Robot.x_des-Robot.x).norm()<<endl;
		}
		else{
			if(arrived){
				Robot->set_robot_state(Robot_States::PICKING_OBJECT);
				control_torques.setZero();
			}
		}
		break;

	case ORIENTING_ARM:
		control_torques.setZero();
		break;

	default:
		control_torques.setZero();
		break;
	}
	//Go to dle when done

	return control_torques;
}



int main() {

	// Make sure redis-server is running at localhost with default port 6379
	// start redis client
	RedisClient redis_client = RedisClient();
	redis_client.connect();

	// set up signal handler
	signal(SIGABRT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGINT, &sighandler);

	// load robots, read current state nd update the model
	auto robot = new Sai2Model::Sai2Model(robot_file, false);
	auto object = new Sai2Model::Sai2Model(obj_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	object->_q =redis_client.getEigenMatrixJSON(OBJ_JOINT_ANGLES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();


	//define variables
	VectorXd control_torques = VectorXd::Zero(robot->dof());
	Simulation_states current_simulation_state=Simulation_states::PICK_SHELF_OBJECTS; //in this state while writting function.

	//objects variables
	Vector3d x_obj;
	object->positionInWorld(x_obj, "link1", Vector3d(0, 0, 0));

	// create a loop timer
	double control_freq = 1000;
	LoopTimer timer;
	timer.setLoopFrequency(control_freq);   // 1 KHz
	double last_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;
	timer.initializeTimer(1000000); // 1 ms pause before starting loop

	unsigned long long counter = 0;






	//START ROBOT
	//get initial condition
	Grocery_Robot Robot;
	Robot.robot=robot; //sai2 model
	Robot.dof = robot->dof();

	robot->positionInWorld(Robot.x_des, Robot.link_name, Robot.pos_in_link);//get initial position
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY); //get current joint posiiton
	robot->rotation(Robot.R_des, Robot.link_name); //current orientation

	//create robot object
	Robot.cur_time = timer.elapsedTime();
	Robot.x_vel_des << 0.0, 0.0, 0.0;//goal velocity
	Robot.x_acc_des << 0.0, 0.0, 0.0;//goal acceleration
	Robot.q_des=robot->_q;//goal joints angle
	Robot.kp = 200.0;      // chose your p gain
	Robot.kv = 30.0;      // chose your d gain
	Robot.kvj=14.0; //joint damping
	Robot.kpj=50.0; //joint p gain
	Robot.V_max=0.5; //max velocity
	//initialize waypoints
	Robot.basket_to_shelf_waypoints = MatrixXd(5,3);
	Robot.basket_to_shelf_waypoints<< 0.5,0.5,0.5,
	0.8,0.8,0.8,
	-0.2,-0.2,0.5,
	1.5,1.0,0.5,
	-1.0,0.5,0.8;
	Robot.basket_to_shelf_waypoints.transposeInPlace();
	Robot.waypoint_iterator = -1;




	runloop = true;
	while (runloop)
	{

fTimerDidSleep = timer.waitForNextLoop();

		// read robot state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		object->_q== redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->updateModel();
		object->updateModel();
		// update robot information (position, vel, J,L etc)
		Robot.Update_states();
		Robot.cur_time = timer.elapsedTime();
		//State machine
		switch (current_simulation_state)
		{
		case Simulation_states::IDLE:
			/* code */
			control_torques.setZero();
			break;
		// case Simulation_states::BETWEEN_WAYPOINTS:
		// 	break;
		case Simulation_states::GO_TO_SHELF:
			control_torques.setZero();
			break;
		case Simulation_states::PICK_SHELF_OBJECTS:
			switch (Robot.Current_state)
			{
			//if robot is idle, go to the next object in the list
			case Robot_States::R_IDLE:
				//define object goal (go trough list)
				//Robot.x_des<< 0, -0.8, 0.50;
				object->positionInWorld(x_obj, "link6", Vector3d(0, 0, 0));
				Robot.x_des=x_obj;
				Robot.q_gripper_goal=0.04;
				Robot.R_des << 0.696707, -0.717356, -7.0252e-12,
					-0.717356, -0.696707, -6.82297e-12,
					0, 9.79318e-12, -1;
				//change robot status to moving to goal
				Robot.set_robot_state(Robot_States::MOVING_ARM);
				break;

			default://keep doing what was doing
				// control_torques=f(Robot);
				break;
			}


			control_torques=pick_shelf_objects(&Robot); //maybe pass object, to check object position?
			break;
		case Simulation_states::GO_TO_CONVEYOR:
			control_torques.setZero();
			break;
		case Simulation_states::PLACE_OBJECS_CONVEYOR:
			control_torques.setZero();
			break;
		default:
			control_torques.setZero();
			break;
		}

		//Compute controller

		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

		counter++;


	}

	control_torques.setZero();
	redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, control_torques);

	double end_time = timer.elapsedTime();
    std::cout << "\n";
    std::cout << "Control Loop run time  : " << end_time << " seconds\n";
    std::cout << "Control Loop updates   : " << timer.elapsedCycles() << "\n";
    std::cout << "Control Loop frequency : " << timer.elapsedCycles()/end_time << "Hz\n";


    return 0;
}
