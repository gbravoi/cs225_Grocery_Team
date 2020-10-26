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
	APROACHING_OBJECT,
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
	//This is the robot class.
	

	public:
		//******list of atributes***********:
		Robot_States Current_state=Robot_States::R_IDLE; //robot state
		//simulation
		Sai2Model::Sai2Model* robot;//robot objetc
		//dynamic properties (could be in world or robot frame, depending on the update function)
		Eigen::MatrixXd J0; //end effector basic Jacobian
		Eigen::MatrixXd L0; //Lambda_0 at end effector
		MatrixXd N0;
		MatrixXd Jv;
		MatrixXd Lv;//lambda asociated to Jv
		MatrixXd J_bar;
		MatrixXd N;
		VectorXd g;
		//current state in robot frame
		Vector3d x;//current position end effector
		Eigen::Matrix3d R;//current orientation
		//world information
		Eigen::Matrix3d R_w;//current orientation end effector in world
		Vector3d x_w;//current position end effector in world

		//other states
		VectorXd q;//current joints angle
		VectorXd dq;//current joints velocity
		
		//note, goals better pass them as argument to controller. that way define to do thing in world or robot frame
		//exep for the grip, we should know its position
		double finger_griper_goal=0.04; //start open
		
		//time, waypoints
		double cur_time;
		MatrixXd basket_to_shelf_waypoints;
		double last_waypoint_arrival_time;
		double max_time_btw_waypoints = 2.0;
		int waypoint_iterator=-1;
		double distance_thres = 0.01;
		
		//control constant
		double kp=200.0;      // chose your p gain
		double kv=30.0;      // chose your d gain
		double kvj=14.0; //joint damping
		double kpj=80.0; //joint p gain
		double Vmax_robot=2.0; //max velocity

		//object to interact
		//Sai2Model::Sai2Model* target_object;//object the robot needs to interact

		//Robot properties
		int dof;
		const string link_name = "link7";
		const Vector3d pos_in_link = Vector3d(0, 0, 0.35);//0.15//adjust this to aproach fine the objects
		const Vector3d robot_offset=Vector3d(0.0,0.3,0.15);



		//functions definitions
		//VectorXd Position_orientation_controller(bool joint_control);
		VectorXd World_Position_Orientation_controller(Vector3d goal_pos_in_world,Matrix3d R_des,double Vmax);
		void Update_states();
		void Update_dynamics(bool world_coordinates);
		void set_robot_state(Robot_States robot_state);
		//bool checkWaypoints(MatrixXd& waypoints, int num_waypoints);
};

void Grocery_Robot::Update_states(){
	// update robot model and compute gravity, and other stuff
	robot->updateModel();
	q=robot->_q;
	dq=robot->_dq;
	robot->position(x, link_name, pos_in_link);
	robot->positionInWorld(x_w, link_name, pos_in_link);
	x_w+=robot_offset;//add offset
	robot->rotation(R, link_name);
	robot->rotationInWorld(R_w, link_name);
 };

 void Grocery_Robot::Update_dynamics(bool world_coordinates=false){
	// update robot model and compute gravity, and other stuff
	robot->updateModel();
	robot->gravityVector(g);

	if( world_coordinates){
		robot->J_0WorldFrame(J0, link_name, pos_in_link);
		robot->JvWorldFrame(Jv, link_name, pos_in_link);

	}else{//else robot coordinates
		robot->J_0(J0, link_name, pos_in_link);
		robot->Jv(Jv, link_name, pos_in_link);
	}

	//these ones depend of the values we computed before
	robot->nullspaceMatrix(N0, J0);
	robot->taskInertiaMatrix(L0, J0);
	robot->taskInertiaMatrix(Lv, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
	
 };

void Grocery_Robot::set_robot_state(Robot_States robot_state){
	//function that update robot state, help to print current robot state to keep track
	Current_state=robot_state;
	cout << "Current Robot State "<<  robot_state <<endl;
};

VectorXd Grocery_Robot::World_Position_Orientation_controller(Vector3d goal_pos_in_world,Matrix3d R_des=Matrix3d::Zero(3,3),double Vmax=0.0){
	/**
	 * takes the end effector to a position. 
	 * can be performed at a reduced max velocity.
	 * designed to move the robot arm to an object using frame coordinates
	**/
	double nu=1;
	VectorXd control_torques;
	Vector3d x_vel_des;
	bool world_coordinates=true;
	VectorXd q_des;
	Vector3d delta_phi;

	//update dynamics in world coordinates
	Update_dynamics(world_coordinates);

	//compute nu of Vmax
	if(Vmax!=0){
		x_vel_des = - kp / kv * (x_w - goal_pos_in_world);
		nu = sat(Vmax / x_vel_des.norm());
	}else{//else use robot restrictions
		x_vel_des = - kp / kv * (x_w - goal_pos_in_world);
		nu = sat(Vmax_robot / x_vel_des.norm());
	}
	//current velocity
	Vector3d x_vel;
	robot->linearVelocityInWorld(x_vel, link_name, pos_in_link);

	//always control the gripper
	q_des=q;//all other joints in current position
	//Change end effector position to the desired position.
	q_des(10)=finger_griper_goal;
	q_des(11)=-1*finger_griper_goal;
	
	
	

	Vector3d pd_x = (- kp * (x_w - goal_pos_in_world)-kv*(x_vel-nu*x_vel_des)); //(- kp * nu * (x_w - goal_pos_in_world));

	//if rotation matrix is 0, only control position, else control position and orientation
	if(R_des==MatrixXd::Zero(3,3)){
		control_torques = Jv.transpose()* Lv*pd_x + N.transpose() *robot->_M *( - (kvj * dq)-kpj*(q-q_des)) + g;  // gravity is compensated in simviz loop as of now
	}else{
		//find error in rotation
		delta_phi = -0.5 * (R_w.col(0).cross(R_des.col(0)) + R_w.col(1).cross(R_des.col(1)) + R_w.col(2).cross(R_des.col(2)));

		//get angular velocity in world coordinate
		Vector3d w;//current angular velocity
		robot->angularVelocityInWorld(w, link_name);

		Vector3d pd_w = kp * (- delta_phi) - kv * w;
		
		VectorXd pd(6);
		pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];
		VectorXd F(6);
		F = L0 * pd;
 		control_torques = J0.transpose() * F + N0.transpose() * robot->_M*( - (kvj *dq) -kpj*(q-q_des)) + 0*g;  // gravity is compensated in simviz loop as of now



	}

	return control_torques;
}





// VectorXd Grocery_Robot::Position_orientation_controller(bool joint_control=false,bool world_coordinates=false){
// 	//control posiiton and orientation based on the goals specified in atributes.
		
// 		Vector3d pos;//auxiliary variable to store position to use in controller
// 		Matrix3d Rot;//auziliary variable to store rotation to use in controller
// 		VectorXd control_torques;
// 		Vector3d delta_phi;

// 		Update_dynamics(world_coordinates);//update dynamics in robot coordinates or world (trus the user to set the goal in the right frame)
// 		//use position and orientation depending on coordinates
// 		if (world_coordinates){
// 			pos=x_w
// 			Rot=R_w
// 		}else{
// 			pos=x
// 			Rot=R
// 		}

// 		delta_phi = -0.5 * (R.col(0).cross(R_des.col(0)) + Rot.col(1).cross(R_des.col(1)) + Rot.col(2).cross(R_des.col(2)));

// 		double Vmax = 0.5;
// 		x_vel_des = - kp / kv * (pos - x_des);
// 		double nu = sat(Vmax / x_vel_des.norm());

// 		//joint desired state
// 		if (joint_control==false){
// 			q_des=robot->_q;//all other joints in current position
// 			}
// 		//Change end effector position to the desired position.
// 		q_des(10)=q_gripper_goal;
// 		q_des(11)=-1*q_gripper_goal;

// 		Vector3d pd_x = - kp * nu * (pos - x_des) - kv * x_vel;
// 		Vector3d pd_w = kp * (- delta_phi) - kv * w;
// 		VectorXd pd(6);
// 		pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];

// 		VectorXd F(6);
// 		F = L0 * pd;
// 		control_torques = J0.transpose() * F + N.transpose() * ( - (kvj * robot->_dq) -robot->_M*kpj*(robot->_q-q_des)) + 0*g;  // gravity is compensated in simviz loop as of now

// 		return control_torques;
//  };

// bool Grocery_Robot::checkWaypoints(MatrixXd& waypoints, int num_waypoints){
// 	if(waypoint_iterator == -1){
// 		waypoint_iterator = 0;
// 		last_waypoint_arrival_time = cur_time;
// 		x_des = waypoints.col(waypoint_iterator);
// 	}

// 	Vector3d dist = x-waypoints.col(waypoint_iterator);
// 	if(dist.norm() < distance_thres || cur_time - last_waypoint_arrival_time > max_time_btw_waypoints){
// 		waypoint_iterator++;
// 		last_waypoint_arrival_time = cur_time;
// 		if(waypoint_iterator >= num_waypoints){
// 			waypoint_iterator = -1;
// 			return true;
// 		}else{
// 			x_des = waypoints.col(waypoint_iterator);
// 		}
// 	}
// 	return false;
// };


//object class
class Objects_class{
	public:
		//simulation
		Sai2Model::Sai2Model* SAI2_object;//SAI2 representation of the object
		const string link_name = "link6";
		const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);

		//current states in world frame
		Eigen::Vector3d x_w;//current position in world
		Eigen::Matrix3d R_w;//current orientation in world
		Vector3d obj_offset;
		double height=0.19;
		double width=0.072898; 
		double deep=0.089916;
	

		//functions definition
		void Update_states();
};
void Objects_class::Update_states(){
	// update object model
	SAI2_object->updateModel();
	SAI2_object->positionInWorld(x_w, link_name, pos_in_link); //later add noise to the position
	x_w+=obj_offset;//add offset

	SAI2_object->rotationInWorld(R_w, link_name);
 };



//---------------------------------------
//-------State machine sub functions-----
//---------------------------------------

VectorXd pick_shelf_objects(Grocery_Robot *Robot, Objects_class *Object){
	VectorXd control_torques= VectorXd::Zero(Robot->dof);
	double control_threshold=0.07;
	bool arrived = false;
	Vector3d target_pos;
	Vector3d x_w;
	Vector3d delta_phi;
	
	double y_safe;
	Matrix3d R_des;
	Matrix3d R_w;

	//define horizontal horientation
	R_des<< 1.57079632679,1.57079632679, 0.0,
		0.0, 0.0, -1.0,
		-1.57079632679, 1.57079632679, 0.0;
	//define positon in the shelf in front of the object
	y_safe=0.5;
	target_pos=Object->x_w;
	target_pos(1)+=y_safe;
	x_w=Robot->x_w;


	switch (Robot->Current_state)
	{
		case MOVING_ARM:
			/**
			 * move arm in front of the shelve.
			 * this will be the heigh of the object, but keeping safe distance from the shelve (y distance)
			 * this could be also reach using way points
			 * **/
			
			
			//cout<<(target_pos-x_w).norm()<<endl;
			//if reached postion, move to orient
			if ((target_pos-x_w).norm()<=control_threshold){
				Robot->set_robot_state(Robot_States::ORIENTING_ARM);
				control_torques.setZero();
			}else{//else move to position
				control_torques=Robot->World_Position_Orientation_controller(target_pos);
				
			}
			
			break;


		case ORIENTING_ARM:
			/**
			 * orient horizontal and open grip
			 * /We want the orientation be horizontal to the plane
			 **/

			R_w=Robot->R_w;
			delta_phi = -0.5 * (R_w.col(0).cross(R_des.col(0)) + R_w.col(1).cross(R_des.col(1)) + R_w.col(2).cross(R_des.col(2)));

			//cout<< delta_phi.norm()<<endl;
			//if oriente, move to aproacing stage
			if (delta_phi.norm()<=1e-4){
				Robot->set_robot_state(Robot_States::APROACHING_OBJECT);
				control_torques.setZero();
			}else{
				control_torques=Robot->World_Position_Orientation_controller(target_pos,R_des);
			}

			break;


		case APROACHING_OBJECT:
			/**
			 * aproach object keeping gripper horizontal
			 **/
			target_pos=Object->x_w;
			target_pos(1)+=Object->width;//correct by the object width

			//if is near object, start closing the gripper
			if ((target_pos-x_w).norm()<=control_threshold){
				Robot->set_robot_state(Robot_States::PICKING_OBJECT);
				control_torques.setZero();
			}else{
				control_torques=Robot->World_Position_Orientation_controller(target_pos,R_des);
			}

			break;



		case PICKING_OBJECT:

			control_torques.setZero();
			break;


		case PLACING_OBJECT:
			//placinf object in the basket.
			// arrived = Robot->checkWaypoints(Robot->basket_to_shelf_waypoints, 5);
		
			// if( (Robot->x_des-Robot->x).norm()>control_threshold){
			// 	control_torques=Robot->Position_orientation_controller(false);
			// 	//cout<<(Robot.x_des-Robot.x).norm()<<endl;
			// }
			// else{
			// 	if(arrived){
			// 		Robot->set_robot_state(Robot_States::PICKING_OBJECT);
			// 		control_torques.setZero();
			// 	}
			// }
			control_torques.setZero();
			break;

		default:
			control_torques.setZero();
			break;
		}
		//Go to dle when done

	return control_torques;
};



int main() {
	//*************INITIALIZATION******************************
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
	auto cup = new Sai2Model::Sai2Model(obj_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	cup->_q =redis_client.getEigenMatrixJSON(OBJ_JOINT_ANGLES_KEY);
	cup->_dq =redis_client.getEigenMatrixJSON(OBJ_JOINT_VELOCITIES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();
	cup->updateModel();


	//define variables
	VectorXd control_torques = VectorXd::Zero(robot->dof());
	Simulation_states current_simulation_state=Simulation_states::PICK_SHELF_OBJECTS; //in this state while writting function.

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

	//create robot object
	Robot.cur_time = timer.elapsedTime();
	//initialize waypoints
	Robot.basket_to_shelf_waypoints = MatrixXd(5,3);
	Robot.basket_to_shelf_waypoints<< 0.5,0.5,0.5,
	0.8,0.8,0.8,
	-0.2,-0.2,0.5,
	1.5,1.0,0.5,
	-1.0,0.5,0.8;
	Robot.basket_to_shelf_waypoints.transposeInPlace();
	Robot.waypoint_iterator = -1;


	//Start CUP
	Objects_class Cup;
	Cup.SAI2_object=cup;
	Cup.obj_offset<<0, -0.35, 0.544+0.095;






	runloop = true;
	while (runloop)
	{

fTimerDidSleep = timer.waitForNextLoop();

		// read robot and object state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		cup->_q= redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->updateModel();
		cup->updateModel();


		// update robot information (position orientation)
		Robot.Update_states();
		Robot.cur_time = timer.elapsedTime();

		//update objets position and orientation
		Cup.Update_states();




		//State machine
		switch (current_simulation_state)
		{
		case Simulation_states::IDLE:
			control_torques.setZero();
			break;
		// case Simulation_states::BETWEEN_WAYPOINTS:
		// 	break;
		case Simulation_states::GO_TO_SHELF:
			//aproach robot platform in front of the shelf
			control_torques.setZero();
			break;
		case Simulation_states::PICK_SHELF_OBJECTS:
			 switch (Robot.Current_state)
			{
			//if robot is idle, go to the next object in the list
			//if list ended, move to go to conveyor
			 case Robot_States::R_IDLE:
				//move to the next object
				Robot.set_robot_state(Robot_States::MOVING_ARM);
				control_torques=pick_shelf_objects(&Robot,&Cup); //maybe pass object, to check object position?
				break;
			 default:
			    //default, continue in the picking shelf function
				control_torques=pick_shelf_objects(&Robot,&Cup); 
			 break;
			}
			


			
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
