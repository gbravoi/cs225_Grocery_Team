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
const string robot_name = "mmp_panda";

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
const std::string EE_FORCE_KEY_r = "cs225a::sensor::force2";
const std::string EE_FORCE_KEY_l = "cs225a::sensor::force3";
const std::string EE_FORCE_KEY_EEF = "cs225a::sensor::force1";
const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";




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
	MOVING_ARM,
	ORIENTING_ARM,
	APROACHING_OBJECT,
	PICKING_OBJECT,
	MOVING_BACKWARDS,
	PLACING_OBJECT,
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
		double height;
		double width; 
		double deep;
	

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
		MatrixXd M; //mass matrix

		//matrices needed for arm control
		MatrixXd Jv_arm;
		MatrixXd Lv_arm;
		MatrixXd N_arm;
		MatrixXd J0_arm;
		MatrixXd L0_arm;
		MatrixXd N0_arm;
		MatrixXd M_arm;

		//states that could be in world or robot frame, updated with dynamics
		Vector3d x_vel;//linear velocity
		Vector3d w;//angular velocity

		//current state in robot frame
		Vector3d x;//current position end effector
		Eigen::Matrix3d R;//current orientation
		//in world frame
		Eigen::Matrix3d R_w;//current orientation end effector in world
		Vector3d x_w;//current position end effector in world

		//other states
		VectorXd q;//current joints angle
		VectorXd dq;//current joints velocity

		//save last base position for the case we want to move it
		Vector3d base_position;
		
		
		//time, waypoints
		double cur_time;
		MatrixXd basket_to_shelf_waypoints;
		double last_waypoint_arrival_time;
		double max_time_btw_waypoints = 2.0;
		int waypoint_iterator=-1;
		double distance_thres = 0.01;
		

		double Vmax_robot=0.5; //max velocity

		//object to interact
		//Sai2Model::Sai2Model* target_object;//object the robot needs to interact

		//Robot properties
		int dof;
		const string link_name = "link7";
		const Vector3d pos_in_link = Vector3d(0, 0, 0.15);//0.18 distance to touch base of the grip.
		const Vector3d robot_offset=Vector3d(0.0,0.3,0.15);//offset in world, given by world urdf
		VectorXd force_finger_r;//measure force by sensor
		VectorXd force_finger_l;//measured force by sensor
		VectorXd force_eef;//force in the link 7 of the end effector
		double open_finger=0.04; //joint position of open finger
		VectorXd HomePosition;



		//functions definitions
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
		robot->linearVelocityInWorld(x_vel, link_name, pos_in_link);
		robot->angularVelocityInWorld(w,link_name);

	}else{//else robot coordinates
		robot->J_0(J0, link_name, pos_in_link);
		robot->Jv(Jv, link_name, pos_in_link);
		robot->linearVelocity(x_vel, link_name, pos_in_link);
		robot->angularVelocity(w, link_name);
	}

	//these ones depend of the values we computed before
	robot->nullspaceMatrix(N0, J0);
	robot->taskInertiaMatrix(L0, J0);
	robot->taskInertiaMatrix(Lv, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);
	M=robot->_M;


	//start arm jacobian 
	Jv_arm=	Jv.block(0,3,3,7);//A.block(i,j,m,n): Returns a matrix containing the block of m rows and n columns whose upper left corner is at (i,j)
	M_arm=M.block(3,3,7,7);
	Lv_arm=(Jv_arm*M_arm.inverse()*Jv_arm.transpose()).inverse();
	MatrixXd Iv=MatrixXd::Zero(3, 3);
	MatrixXd Jv_bar;
	Jv_bar=M_arm.inverse()*Jv_arm.transpose()*Lv_arm;
	N_arm=Iv-Jv_arm.transpose()*Jv_bar.transpose();
	
	J0_arm=	J0.block(0,3,6,7);//A.block(i,j,m,n): Returns a matrix containing the block of m rows and n columns whose upper left corner is at (i,j)
	L0_arm=(J0_arm*M_arm.inverse()*J0_arm.transpose()).inverse();
	MatrixXd I0=MatrixXd::Zero(6, 6);
	MatrixXd J0_bar;
	J0_bar=M_arm.inverse()*J0_arm.transpose()*L0_arm;
	N0_arm=I0-J0_arm.transpose()*J0_bar.transpose();
	
 };

void Grocery_Robot::set_robot_state(Robot_States robot_state){
	//function that update robot state, help to print current robot state to keep track
	Current_state=robot_state;
	cout << "Current Robot State "<<  robot_state <<endl;
};


//#######Controllers####
/**
 * We can control independently the base, the arm and the finger.
 * Then we stack the torques in one vector to pass to the simulator
 * */

class Controller{
	public:
		Grocery_Robot* Robot;
		Vector2d finger_torques;
		VectorXd Whole_body_controller; //used when robot is picking bjects
		//control constant
		double kp=200.0;      // chose your p gain
		double kv=30.0;      // chose your d gain
		double kvj=14.0; //joint damping
		double kpj=80.0; //joint p gain


		//functions declaration
		void Arm_World_Position_Orientation_controller(Vector3d goal_pos_in_world,Matrix3d R_des, double Vmax);
		void Arm_Local_Position_Orientation_controller(Vector3d goal_pos_in_base,Matrix3d R_des, double Vmax);
		void Gripper_controller(double gripper_force);
		void Base_controller(Vector3d Position,double Vmax);
		VectorXd Return_torques();
		

};
void Controller::Arm_World_Position_Orientation_controller(Vector3d goal_pos_in_world,Matrix3d R_des=Matrix3d::Zero(3,3), double Vmax=0.0){
	/**
	 *This controller will control all robot to reach object.
	 Moving platfor a little is needed to achieve the right orientation
	 **/
	double nu=1;
	VectorXd control_torques;
	Vector3d x_vel_des;
	bool world_coordinates=true;
	VectorXd q_des;
	Vector3d delta_phi;

	//update dynamics
	Robot->Update_dynamics(world_coordinates);
	MatrixXd Jv=Robot->Jv;
	MatrixXd Lv=Robot->Lv;
	MatrixXd N=Robot->N;
	MatrixXd J0=Robot->J0;
	MatrixXd L0=Robot->L0;
	MatrixXd N0=Robot->N0;
	VectorXd dq=Robot->dq;
	VectorXd q=Robot->q;
	VectorXd x_w=Robot->x_w;
	VectorXd w=Robot->w;
	VectorXd x_vel=Robot->x_vel;
	Matrix3d R_w=Robot->R_w;
	MatrixXd M=Robot->M;

	//compute nu of Vmax
	if(Vmax!=0){
		x_vel_des = - kp / kv * (x_w - goal_pos_in_world);
		nu = sat(Vmax / x_vel_des.norm());
	}else{//else use robot restrictions
		x_vel_des = - kp / kv * (x_w - goal_pos_in_world);
		nu = sat(Robot->Vmax_robot / x_vel_des.norm());
	}


	q_des=q;//try to keep joints in current position
	

	Vector3d pd_x = (-kv*(x_vel-nu*x_vel_des)); //(- kp * nu * (x_w - goal_pos_in_world));

	//if rotation matrix is 0, only control position, else control position and orientation
	if(R_des==MatrixXd::Zero(3,3)){
		control_torques = Jv.transpose()* Lv*pd_x + N.transpose() *M *( - (kvj * dq)-kpj*(q-q_des)) ;  // gravity is compensated in simviz loop as of now
	}else{
		//find error in rotation
		delta_phi = -0.5 * (R_w.col(0).cross(R_des.col(0)) + R_w.col(1).cross(R_des.col(1)) + R_w.col(2).cross(R_des.col(2)));

		Vector3d pd_w = kp * (- delta_phi) - kv * w;
		
		VectorXd pd(6);
		pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];
		VectorXd F(6);
		F = L0 * pd;
 		control_torques = J0.transpose() * F + N0.transpose() * M*( - (kvj *dq) -kpj*(q-q_des)) ;  // gravity is compensated in simviz loop as of now

	}
	Whole_body_controller=control_torques;
};
void Controller::Arm_Local_Position_Orientation_controller(Vector3d goal_pos_in_base,Matrix3d R_des=Matrix3d::Zero(3,3), double Vmax=0.0){
	/**
	 *This controller will only move the arm
	 *positions hsould be described in the base frame of the robot
	 **/
	VectorXd control_torques=VectorXd::Zero(Robot->dof);
	VectorXd arm_controller;
	VectorXd base_controller;
	double nu=1;
	Vector3d x_vel_des;
	bool world_coordinates=false;
	VectorXd q_des;
	Vector3d delta_phi;

	//update dynamics in world coordinates
	Robot->Update_dynamics();//update in base coordinates
	MatrixXd Jv=Robot->Jv_arm;
	MatrixXd Lv=Robot->Lv_arm;
	MatrixXd N=Robot->N_arm;
	MatrixXd J0=Robot->J0_arm;
	MatrixXd L0=Robot->L0_arm;
	MatrixXd N0=Robot->N0_arm;
	MatrixXd M=Robot->M_arm;
	VectorXd dq_arm=Robot->dq.segment(3,7);



	//compute nu of Vmax
	if(Vmax!=0){
		x_vel_des = - kp / kv * (Robot->x - goal_pos_in_base);
		nu = sat(Vmax / x_vel_des.norm());
	}else{//else use robot restrictions
		x_vel_des = - kp / kv * (Robot->x - goal_pos_in_base);
		nu = sat(Robot->Vmax_robot / x_vel_des.norm());
	}
	

	Vector3d pd_x = (-kv*(Robot->x_vel-nu*x_vel_des)); 
	

	//compute arm controller (7 joints)
	//if rotation matrix is 0, only control position, else control position and orientation
	if(R_des==MatrixXd::Zero(3,3)){
		arm_controller = Jv.transpose()* Lv*pd_x + N.transpose() *M *( - (kvj * dq_arm)) ;
	}
	else{
		//find error in rotation
		delta_phi = -0.5 * (Robot->R.col(0).cross(R_des.col(0)) + Robot->R.col(1).cross(R_des.col(1)) + Robot->R.col(2).cross(R_des.col(2)));


		Vector3d pd_w = kp * (- delta_phi) - kv * Robot->w;
		
		VectorXd pd(6);
		pd << pd_x[0], pd_x[1], pd_x[2], pd_w[0], pd_w[1], pd_w[2];
		VectorXd F(6);
		F = Robot->L0 * pd;
 		arm_controller = J0.transpose() * F + N0.transpose() * M*( - (kvj *dq_arm) );
	}

	// //compute base controller to keep corrent position
	Vector3d base_position;
	base_position=Robot->base_position;

	VectorXd q=Robot->q.segment(0,3);
	VectorXd dq=Robot->dq.segment(0,3);
	MatrixXd M_base=Robot->M.block(0,0,3,3);

	base_controller=M_base*(- (kvj *dq) -kpj*(q-base_position));

	control_torques<<base_controller(0),base_controller(1),base_controller(2),arm_controller(0),arm_controller(1),arm_controller(2),arm_controller(3),arm_controller(4),arm_controller(5),arm_controller(6),0.0,0.0;
	Whole_body_controller=control_torques;
};

void Controller::Gripper_controller(double gripper_force){
	/**
	 * Gripper force =0 open,
	 * else, close with that force
	 **/

	//open using a direct PID over joint
	Vector2d q;
	Vector2d dq;
	Vector2d q_des;
	q<<Robot->q(10),Robot->q(11);
	dq<<Robot->dq(10),Robot->dq(11);
	q_des<<Robot->open_finger, -1*Robot->open_finger;
	if (gripper_force==0){
		finger_torques=- (kvj *dq) -kpj*(q-q_des);
	}else{
		finger_torques<<-gripper_force,gripper_force;
	}

};
void Controller::Base_controller(Vector3d Position=VectorXd::Zero(3),double Vmax=0){
	/**
	 * move base to the x,y,theta position while keeping the arm Home position
	 * if new_position=false, create torques to keep current position.
	 * Only call it if need to move base, otherwise this value is 0
	 **/

	Robot->Update_dynamics();//update in base coordinates
	//control position using a direct PID over joint
	VectorXd q;
	VectorXd dq;
	VectorXd q_des;
	VectorXd dq_des;
	q=Robot->q;
	dq=Robot->dq;
	double nu;
	
	q_des=Robot->HomePosition;
	q_des<<Position(0),Position(1),Position(2);
	

	if(Vmax!=0){
		dq_des = - kp / kv * (q - q_des);
		nu = sat(Vmax / dq_des.norm());
		Whole_body_controller=Robot->M*(- (kvj *(dq-nu*dq_des)));
	}else{//else do not limit speed
		Whole_body_controller=Robot->M*(- (kvj *dq) -kpj*(q-q_des));
	}

};
VectorXd Controller::Return_torques(){
	/**
	 * This function combine all controllers to return the final comanded torque.
	 **/
	VectorXd control_torques = VectorXd::Zero(Robot->dof);
	
	//inf force on gripper is 0, compute open gripper
	if (finger_torques==VectorXd::Zero(2)){
		Gripper_controller(0);
	}

	control_torques<<Whole_body_controller[0],Whole_body_controller[1],Whole_body_controller[2],Whole_body_controller[3],Whole_body_controller[4],Whole_body_controller[5],Whole_body_controller[6],Whole_body_controller[7],Whole_body_controller[8],Whole_body_controller[9],finger_torques[0],finger_torques[1];

	
		
	//clear previos torque commands
	finger_torques.setZero();
	Whole_body_controller.setZero();


	return control_torques;
};








//---------------------------------------
//-------State machine sub functions-----
//---------------------------------------

//************pick_shelf_objects*****

double pick_shelf_objects_counter=0;
Vector3d ref_pos;//used when neede to keep the old position of the object
VectorXd pick_shelf_objects(Controller *RobotController , Objects_class *Object){
	
	Grocery_Robot* Robot=RobotController->Robot;
	VectorXd control_torques= VectorXd::Zero(Robot->dof);
	
	double control_threshold=0.07;
	bool arrived = false;
	Vector3d target_pos;
	Vector3d x_w;
	Vector3d delta_phi;
	
	Matrix3d R_des;
	Matrix3d R_w;
	VectorXd force_r;
	VectorXd force_l;
	VectorXd force_ee;
	Vector3d base_initial_pos;
	Vector3d point_in_front_shelf;
	point_in_front_shelf<<0.001563,-0.1,0.67;//this need to be computed, will depend on the orientation or the shelf. Maybe determine a point where to start and return

	
	double force=1;

	//define horizontal horientation
	R_des<< 1.57079632679,1.57079632679, 0.0,
		0.0, 0.0, -1.0,
		-1.57079632679, 1.57079632679, 0.0;

	x_w=Robot->x_w;
	base_initial_pos<<Robot->q(0),Robot->q(1),Robot->q(2);
	 


	switch (Robot->Current_state)
	{
		case MOVING_ARM:
			/**
			 * move arm in front of the shelve.
			 * this will be the heigh of the object, but keeping safe distance from the shelve (y distance)
			 * this could be also reach using way points
			 * **/
			
			//targert position a little far away of the object, in front
			target_pos=point_in_front_shelf;
			target_pos(2)=Object->x_w[2];//same height as object.
			//cout<<(target_pos-x_w).norm()<<endl;
			//if reached postion, move to orient
			if ((target_pos-x_w).norm()<=1e-1){
				Robot->set_robot_state(Robot_States::ORIENTING_ARM);
				control_torques.setZero();
			}else{//else move to position
				RobotController->Arm_World_Position_Orientation_controller(target_pos);//control arm and platform, need both to ensure position
				RobotController->Gripper_controller(0);//gripper open
				control_torques=RobotController->Return_torques();
				
			}
			
			break;


		case ORIENTING_ARM:
			/**
			 * orient horizontal and open grip
			 * /We want the orientation be horizontal to the plane
			 **/
			//same target position as moving arm
			target_pos<< Object->x_w[0],Object->x_w[1],Object->x_w[2];
			target_pos(1)+=0.2;

			//compute error in rotation
			R_w=Robot->R_w;
			delta_phi = -0.5 * (R_w.col(0).cross(R_des.col(0)) + R_w.col(1).cross(R_des.col(1)) + R_w.col(2).cross(R_des.col(2)));

			//cout<< delta_phi.norm()<<endl;
			//if oriented, move to aproacing object
			if (delta_phi.norm()<=1e-4){
				Robot->set_robot_state(Robot_States::APROACHING_OBJECT);
				control_torques.setZero();
			}else{
				RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des);
				RobotController->Gripper_controller(0);
				control_torques=RobotController->Return_torques();
			}

			break;


		case APROACHING_OBJECT:
			/**
			 * aproach object keeping gripper horizontal
			 **/
			target_pos<< Object->x_w[0],Object->x_w[1],Object->x_w[2];
			force_ee=Robot->force_eef;
			//cout<<force_ee.transpose()<<endl;
			if ((force_ee).norm()>1e-5){//as soon it senses object, go next
				Robot->set_robot_state(Robot_States::PICKING_OBJECT);
				control_torques.setZero();
			}else{
				//continue control to aproach
				RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
				RobotController->Gripper_controller(0);
				control_torques=RobotController->Return_torques();
			}

			break;



		case PICKING_OBJECT:
			/**
			 * Close grip to hold object
			 **/	
			force_r=Robot->force_finger_r;
			force_r=Robot->force_finger_l;
			//cout<<"r:"<<force_r.norm()<<" l"<< force_l.norm()<<endl;
			//cout<<Robot->q(11)<<endl;

			//when force detected, hold for a momnet (counter time) then rise, and go next state
			if(force_r.norm()>0.01 || force_l.norm()>0.01){
				if (pick_shelf_objects_counter>2000){
					//move a little up
					target_pos=ref_pos;
					cout<<"target"<<target_pos.transpose()<<endl;
					//cout<<"current"<<x_w.transpose()<<endl;
					target_pos(2)+=0.1;//this is the distance that will go up, latter replace depending on the shelf.
					if ((target_pos-x_w).norm()<=control_threshold){
						Robot->set_robot_state(Robot_States::MOVING_BACKWARDS);
						RobotController->Arm_World_Position_Orientation_controller(ref_pos,R_des,0.1);
						RobotController->Gripper_controller(force);//gripper will hold in the designated force
						control_torques=RobotController->Return_torques();
						ref_pos<< x_w[0],x_w[1],x_w[2];
						pick_shelf_objects_counter=0;//reset counter
					}else{
						RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
						RobotController->Gripper_controller(force);//gripper will hold in the designated force
						control_torques=RobotController->Return_torques();	
					}
				}else{
					pick_shelf_objects_counter+=1;
					cout<< "counter "<< pick_shelf_objects_counter <<endl;
					control_torques(11)=force;
					control_torques(10)=-force;
				}
			}else{
				//hold for a moment in current position holding the objetc
				RobotController->Gripper_controller(force);//gripper will hold in the designated force
				control_torques=RobotController->Return_torques();
				//save current position as reference
				ref_pos<< x_w[0],x_w[1],x_w[2];
			}
			break;


		case MOVING_BACKWARDS:
			/**
			 * Move backward to leave shelf space.
			 **/
			//here must use a waypoint, should be the position in front of the shelf
			target_pos=point_in_front_shelf;
			target_pos(2)=ref_pos(2);//keep same height
						
			if ((target_pos-x_w).norm()<=control_threshold){
				//Robot->set_robot_state(Robot_States::PLACING_OBJECT);
				RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
				RobotController->Gripper_controller(force);//gripper will hold in the designated force
				control_torques=RobotController->Return_torques();
			}else{
				RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
				RobotController->Gripper_controller(force);//gripper will hold in the designated force
				control_torques=RobotController->Return_torques();
			}

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


//************Navigate to a point*****
VectorXd Navigate_to_point(Controller *RobotController , Vector3d point_world, double V_max=0.0){
	/**
	 * Auxiliary function to navegate to a point,
	 * gives the needed torques and perfr=orm other operations needed like log the last platform position.
	 **/
	Grocery_Robot* Robot=RobotController->Robot;
	RobotController->Base_controller(point_world,V_max);
	//upodate last position of the base
	Robot->base_position<<Robot->q(0),Robot->q(1),Robot->q(2);
	//send controller
	return RobotController->Return_torques();
}


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
	Robot.HomePosition=VectorXd::Zero(Robot.dof);
	Robot.HomePosition<<0.0,0.0,0.0, -0.798855 ,-0.328792, -0.001426 ,-0.006871, -0.000757, -0.053838 ,-0.000491 ,-0.039092, -0.119215;
	Robot.Update_states();
	Robot.base_position<<Robot.q(0),Robot.q(1),Robot.q(2);


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


	//start robot controller
	Controller RobotController;
	RobotController.Robot=&Robot;
	//start with 0 torques
	RobotController.finger_torques=VectorXd::Zero(2);
	RobotController.Whole_body_controller=VectorXd::Zero(robot->dof()); 





	//Start CUP
	Objects_class Cup;
	Cup.SAI2_object=cup;
	Cup.obj_offset<<0, -0.35, 0.593+0.19/2;
	Cup.height=0.19;
	Cup.width=0.072898; 
	Cup.deep=0.089916;






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
		//update force sensor
		Robot.force_finger_r=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_r);
		Robot.force_finger_l=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_l);
		Robot.force_eef=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_EEF);


		// update robot information (position orientation)
		Robot.Update_states();
		Robot.cur_time = timer.elapsedTime();

		//update objets position and orientation
		Cup.Update_states();


		//neede variables for state machine
		Vector3d Position;
		double Nav_Vmax;

		

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
			//move platform with arm in home position.
			Position<<1.0,1.0,0.0;//this shoudl be the positon of the shelf
			Nav_Vmax=0.5;//local maximum velocity
			control_torques=Navigate_to_point(&RobotController , Position,Nav_Vmax);
			//control_torques.setZero();
			break;
		case Simulation_states::PICK_SHELF_OBJECTS:
			 switch (Robot.Current_state)
			{
			//if robot is idle, go to the next object in the list
			//if list ended, move to go to conveyor
			 case Robot_States::R_IDLE:
				//move to the next object
				//Robot.set_robot_state(Robot_States::MOVING_ARM);
				//control_torques=pick_shelf_objects(&RobotController,&Cup); //maybe pass object, to check object position?
				//Arm_Local_Position_Orientation_controller(Vector3d goal_pos_in_base,Matrix3d R_des=Matrix3d::Zero(3,3), double Vmax=0.0)
				
				
				
				Position<<0.2,0.2,0.2;//this shoudl be the positon of the shelf
				RobotController.Arm_Local_Position_Orientation_controller(Position);
				control_torques=RobotController.Return_torques();
				

			// 	Position<<1.0,1.0,0.0;//this shoudl be the positon of the shelf
			// Nav_Vmax=0.5;//local maximum velocity
			// control_torques=Navigate_to_point(&RobotController , Position,Nav_Vmax);


				// cout<<control_torques.transpose()<<endl;
				// cout<<Robot.q.transpose()<<endl;

				

				break;
			 default:
			    //default, continue in the picking shelf function
				control_torques=pick_shelf_objects(&RobotController,&Cup); 
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
