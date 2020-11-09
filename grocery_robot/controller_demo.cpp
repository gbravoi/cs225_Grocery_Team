#include <Sai2Model.h>
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <iostream>
#include <string>
#include <list> // for list operations
#include <math.h>
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


//------------------------------------------
//-------------GLOBAL PARAMETERS-----------
//------------------------------------------
// Location of URDF files specifying world and robot information
const string robot_file = "./resources/mmp_panda.urdf";
const string object_file = "./resources/jar.urdf";
const string object2_file = "./resources/jar.urdf";
const string object3_file = "./resources/jar.urdf";
const string robot_name = "mmp_panda";

// Redis is just a key value store, publish/subscribe is also possible
// The visualizer and simulator will have keys like "cs225a::robot::{ROBOTNAME}::sensors::q"
// You can hardcode the robot name in like below or read them in from cli
// redis keys:
// - read:
const std::string JOINT_ANGLES_KEY  = "cs225a::robot::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "cs225a::robot::panda::sensors::dq";
// - write:
const std::string JOINT_TORQUES_COMMANDED_KEY  = "cs225a::robot::panda::actuators::fgc";
const std::string EE_FORCE_KEY_r = "cs225a::sensor::force2";
const std::string EE_FORCE_KEY_l = "cs225a::sensor::force3";
const std::string EE_FORCE_KEY_EEF = "cs225a::sensor::force1";
const std::string EE_MOMENT_KEY = "cs225a::sensor::moment";

//MOD FOR SEVERAL OBJECTS
const std::string OBJECT1_JOINT_ANGLES_KEY  = "cs225a::object::object1::sensors::q";
const std::string OBJECT1_JOINT_VELOCITIES_KEY = "cs225a::object::object1::sensors::dq";
const std::string OBJECT2_JOINT_ANGLES_KEY  = "cs225a::object::object2::sensors::q";
const std::string OBJECT2_JOINT_VELOCITIES_KEY = "cs225a::object::object2::sensors::dq";
const std::string OBJECT3_JOINT_ANGLES_KEY  = "cs225a::object::object3::sensors::q";
const std::string OBJECT3_JOINT_VELOCITIES_KEY = "cs225a::object::object3::sensors::dq";

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
	NAVIGATING};

std::ostream& operator<<(std::ostream& out, const Robot_States value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(R_IDLE);
        PROCESS_VAL(MOVING_ARM);
				PROCESS_VAL(ORIENTING_ARM);
				PROCESS_VAL(APROACHING_OBJECT);
				PROCESS_VAL(PICKING_OBJECT);
				PROCESS_VAL(MOVING_BACKWARDS);
				PROCESS_VAL(PLACING_OBJECT);
				PROCESS_VAL(NAVIGATING);
    }
#undef PROCESS_VAL
    return out << s;
}
std::ostream& operator<<(std::ostream& out, const Simulation_states value){
    const char* s = 0;
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(IDLE);
        PROCESS_VAL(GO_TO_SHELF);
				PROCESS_VAL(PICK_SHELF_OBJECTS);
				PROCESS_VAL(GO_TO_CONVEYOR);
				PROCESS_VAL(PLACE_OBJECS_CONVEYOR);
    }
#undef PROCESS_VAL
    return out << s;
}


//other values
const double SHELF_SAFE_DIST=0.1;
Simulation_states current_simulation_state=Simulation_states::GO_TO_SHELF; //in this state while writting function.



//------------------------------------------
//-------------OTHER FUNCTIONS-----------
//------------------------------------------
void set_simulation_state(Simulation_states new_state){
	current_simulation_state=new_state;
	cout << "Current Simulation State "<<  new_state <<endl;
}
Matrix3d Rot_z_matrix(double angle){
	Matrix3d Rot;
	Rot<<cos(angle), -sin(angle),0,
			sin(angle),cos(angle),0,
			0,0,1;
	return Rot;
}
Matrix3d Rot_y_matrix(double angle){
	Matrix3d Rot;
	Rot<<cos(angle), 0,sin(angle),
		0,1,0
		-sin(angle),0,cos(angle);
	return Rot;
}
Matrix3d Rot_x_matrix(double angle){
	Matrix3d Rot;
	Rot<<1,0,0,
		0, cos(angle), -sin(angle),
		0,sin(angle),cos(angle);
	return Rot;
}
const Matrix3d R_hor_ee=Rot_x_matrix(-1.57079632679)*Rot_z_matrix(-0.78539816339);//end effector horizontal
const Matrix3d R_down_ee=Rot_x_matrix(-3.14)*Rot_z_matrix(-0.78539816339);//end efector looking down

//------------------------------------------
//-------------CLASSES DEFINITION-----------
//------------------------------------------
//**********SHELF*******************
class Shelf{
	public:
		Vector3d origin_xyz;//position
		Vector3d origin_rpy;//orientation
		double depth; //depth of the shelf

		//functions
		Matrix3d get_eef_orientation();
		Vector3d position_front_shelf();
		Matrix3d get_Rot_matrix();

};
Matrix3d Shelf::get_Rot_matrix(){
	double angle=origin_rpy(2);//rotation in z
	Matrix3d Rot=Rot_z_matrix(angle);
	return Rot;

}
Matrix3d Shelf::get_eef_orientation(){
	//return orientation needed for the robot end effector face the shelf
	Matrix3d Rot=get_Rot_matrix();
	return Rot_z_matrix(-3.14159265359)*Rot*R_hor_ee;
};
 Vector3d Shelf::position_front_shelf(){
	 //distance is in the y direction of the shelf
	 Vector3d y_distance;
	 y_distance<<0, SHELF_SAFE_DIST+depth,0;

	 //rotation in world
	Matrix3d Rot=get_Rot_matrix();
	return origin_xyz+Rot*y_distance;
 };



//************OBJECT**********************
class Objects_class{
	/**
	 * Class to represent objects that the robot willl interact
	 **/
	public:
		//simulation
		Sai2Model::Sai2Model* SAI2_object;//SAI2 representation of the object
		const string link_name = "link6";
		const Vector3d pos_in_link = Vector3d(0.0, 0.0, 0.0);

		//current states in world frame
		Eigen::Vector3d x_w;//current position in world
		Eigen::Matrix3d R_w;//current orientation in world
		Vector3d obj_offset;
		string name;//name of the object
		double height;
		double width;
		double depth;
		Shelf shelf;//shelf where the object is located


		//functions definition
		void Update_states();
		Vector3d get_position_front_object_in_shelf();

};
void Objects_class::Update_states(){
	// update object model
	SAI2_object->updateModel();
	SAI2_object->positionInWorld(x_w, link_name, pos_in_link); //later add noise to the position
	x_w+=obj_offset;//add offset

	SAI2_object->rotationInWorld(R_w, link_name);
 };
 Vector3d Objects_class::get_position_front_object_in_shelf(){
	 //Return a point at a safe distance of the shelf, but in front of the object
	 Vector3d shelf_pos=shelf.position_front_shelf();
	 shelf_pos(2)=x_w(2);//same height
	 Vector3d diff=shelf_pos-x_w;
	 Matrix3d Rot=shelf.get_Rot_matrix();
	 Vector3d y;
	 y<<0,1,0;
	 Vector3d y_hat_shelf=Rot*y; //y direction (unitary vector) of shelf in world coordinates
	 double proj=diff.dot(y_hat_shelf);//projection
	 return x_w+y_hat_shelf*proj;
 }





//************GROCERY ROBOT**********************
class Grocery_Robot{
	/**
	 * Class the represent the robot, its state and dynamics
	 **/
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

		//aisle properties
		MatrixXd aisle_pts;

		//matrices needed for arm control
		MatrixXd Jv_arm;
		MatrixXd Lv_arm;
		MatrixXd N_arm;
		MatrixXd J0_arm;
		MatrixXd L0_arm;
		MatrixXd N0_arm;
		MatrixXd M_arm;
		Eigen::Affine3d Transformation_w2b;//transformation matrix from world coordinates to base coordinates

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
		Vector3d base_position;//save last base position for the case we want to move arm while keeping that posiiton. Done by navigation state function.
		Vector2d finger_position;//save finger position when holding an object

		//time, waypoints
		double cur_time;
		MatrixXd home_to_basket_waypoints;
		MatrixXd navigation_waypoints;
		double last_waypoint_arrival_time;
		int waypoint_iterator=-1;
		VectorXd next_waypoint_ee;
		Vector3d next_waypoint_base;
		double Vmax_robot=0.5; //max velocity

		//Robot properties
		int dof;
		const string link_name = "link7";
		const Vector3d pos_in_link = Vector3d(0, 0, 0.15);//0.18 distance to touch base of the grip.
		const Vector3d robot_offset=Vector3d(0.0,0.0,0.15);//offset in world, given by world urdf
		const VectorXd home_q = (VectorXd(7)<<-1.5,0.0,0.0,-0.707,0.0,0.707,0.707).finished();
		const VectorXd basket_top_q = (VectorXd(7)<<-1.5,-1.2,0.0,-3.0,0.0,1.8,0.707).finished();
		const double open_finger=0.06; //joint position of open finger
		double pick_up_force=1;
		VectorXd HomePosition;//position keeped by the robot while navigating
		// VectorXd basket_top_q = VectorXd(7);
		// home_q << 0.0,0.0,0.0,0.0,0.0,0.0,0.0;//ee home in robot coordinate
		// basket_top_q << 0.1,0.1,0.1,0.1,0.1,0.1,0.1;//ee home in robot coordinate
		VectorXd force_finger_r;//measure force by sensor
		VectorXd force_finger_l;//measured force by sensor
		VectorXd force_eef;//force in the link 7 of the end effector


		//functions definitions
		void Update_states();
		void Update_dynamics(bool world_coordinates);
		void set_robot_state(Robot_States robot_state);
		void generate_to_basket_waypoints();
		bool checkWaypoints(MatrixXd& waypoints, int num_waypoints, bool ee);
		Matrix3d Rot_h_ee_basket();
		Matrix3d get_base_rotation();
};

void Grocery_Robot::Update_states(){
	// update robot position and orientation i base and world frame
	robot->updateModel();
	q=robot->_q;
	dq=robot->_dq;
	robot->position(x, link_name, pos_in_link);
	robot->positionInWorld(x_w, link_name, pos_in_link);
	x_w+=robot_offset;//add offset
	robot->rotation(R, link_name);
	robot->rotationInWorld(R_w, link_name);
	Transformation_w2b=robot->_T_world_robot;
	base_position<<q(0),q(1),q(2);
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

void Grocery_Robot::generate_to_basket_waypoints(){
	// // home_to_basket_waypoints.col(0) = (home_q+q.segment(3,7))/2;
	// home_to_basket_waypoints.col(0) = home_q;
	// home_to_basket_waypoints.col(1) = basket_top_q;
	// // home_to_basket_waypoints.col(1) = (home_q+basket_top_q)/2;
	// home_to_basket_waypoints.col(2) = basket_top_q;


	//basket frame
	home_to_basket_waypoints.col(0)<<0.15,0,0.7;//POSITION OF THE TOP OF THE BASKET
	home_to_basket_waypoints.col(1)<<0.15,0,0.4;//POSITION OF THE TOP OF THE BASKET
};

void Grocery_Robot::set_robot_state(Robot_States robot_state){
	//function that update robot state, help to print current robot state to keep track
	Current_state=robot_state;
	cout << "Current Robot State "<<  robot_state <<endl;
	if(robot_state==Robot_States::PLACING_OBJECT){
	 	generate_to_basket_waypoints();
	}
};
bool Grocery_Robot::checkWaypoints(MatrixXd& waypoints, int num_waypoints, bool ee){
	/**
	 * Function that ensures the robot follow a set of waypoints.
	 * These points are in baseframe
	 **/
	if(waypoint_iterator == -1){
		waypoint_iterator = 0;
		last_waypoint_arrival_time = cur_time;
		if(ee){
			next_waypoint_ee = waypoints.col(waypoint_iterator);
		}else{
			next_waypoint_base = waypoints.col(waypoint_iterator);
		}
		cout<<"New Waypoint Set"<<endl;
		cout<<"First Waypoint: "<<waypoints.col(waypoint_iterator).transpose()<<endl;
	}
	Vector3d dist;
	if(ee){
		//dist = q-waypoints.col(waypoint_iterator);
		dist=x-waypoints.col(waypoint_iterator);
	}else{
		dist = base_position-waypoints.col(waypoint_iterator);
 	}
	double max_time_btw_waypoints = ee?2.0:20.0;
	double distance_thres = ee?0.01:0.1;
	if(dist.norm() < distance_thres || cur_time - last_waypoint_arrival_time > max_time_btw_waypoints){
		waypoint_iterator++;
		last_waypoint_arrival_time = cur_time;
		if(waypoint_iterator >= num_waypoints){
			waypoint_iterator = -1;
			cout<<"Last waypoint arrived. Error: "<<dist.norm()<<endl;
			return true;
		}else{
			if(ee){
				next_waypoint_ee = waypoints.col(waypoint_iterator);
			}else{
				next_waypoint_base = waypoints.col(waypoint_iterator);
			}
			cout<<"Error: "<<dist.norm()<<" Next Waypoint: "<<waypoints.col(waypoint_iterator).transpose()<<endl;
		}
	}
	return false;
};
Matrix3d Grocery_Robot::Rot_h_ee_basket(){
	/**
	 * Rotation matrix that gives orientation looking to the basket horizontally
	 **/
	return Rot_z_matrix(-1.57)*R_hor_ee;
};

Matrix3d Grocery_Robot::get_base_rotation(){
	/**
	 * Rotation matrix that gives orientation of the base frame
	 **/
	return Rot_z_matrix(q(2));//theta of the base
}
//#######Controllers####
/**
 * We can control independently the base, the arm in base frame, the arm in world frame (this will also move the base) and gripper.
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
		void Arm_Joint_controller(VectorXd q_des);
		void Gripper_controller(double gripper_force);
		void Base_controller(Vector3d Position,double Vmax);
		VectorXd Return_torques();


};
void Controller::Arm_World_Position_Orientation_controller(Vector3d goal_pos_in_world,Matrix3d R_des=Matrix3d::Zero(3,3), double Vmax=0.0){
	/**
	 *This controller will control all robot to reach an object.
	 this control is in world frame
	 Moving platfor a little is needed to achieve the right orientation/posiition
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
		Vector3d pd_w;

		//compute nu of Vmax for delta phi
		if(Vmax!=0){
			x_vel_des = - kp / kv * (delta_phi);
			nu = sat(Vmax / x_vel_des.norm());
			pd_w = (-kv*(Robot->w-nu*x_vel_des));//kp * (- delta_phi) - kv * Robot->w;
		}else{//else typical control
			pd_w = kp * (- delta_phi) - kv * Robot->w;
		}


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
	 *positions should be described in the base frame of the robot
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
		Vector3d pd_w;

		//compute nu of Vmax for delta phi
		if(Vmax!=0){
			x_vel_des = - kp / kv * (delta_phi);
			nu = sat(Vmax / x_vel_des.norm());
			pd_w = (-kv*(Robot->w-nu*x_vel_des));//kp * (- delta_phi) - kv * Robot->w;
		}else{//else typical control
			pd_w = kp * (- delta_phi) - kv * Robot->w;
		}



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
void Controller::Arm_Joint_controller(VectorXd q_des){
	/**
	 *Joint position PD controller
	 **/
	Robot->Update_dynamics();//update in base coordinates
	VectorXd control_torques=VectorXd::Zero(Robot->dof);
	VectorXd arm_controller;
	VectorXd base_controller;
	MatrixXd M_base=Robot->M.block(0,0,3,3);
	//update dynamics in world coordinates
	VectorXd dq_arm=Robot->dq.segment(3,7);
	//compute arm controller (7 joints)
	VectorXd q_arm=Robot->q.segment(3,7);
	arm_controller = -400.0*(q_arm-q_des)-55.0*dq_arm;
	// //compute base controller to keep corrent position
	Vector3d base_position;
	base_position=Robot->base_position;
	VectorXd q_base=Robot->q.segment(0,3);
	VectorXd dq_base=Robot->dq.segment(0,3);
	base_controller=M_base*(- (kvj *dq_base) -kpj*(q_base-base_position));
	control_torques<<base_controller(0),base_controller(1),base_controller(2),arm_controller(0),arm_controller(1),arm_controller(2),arm_controller(3),arm_controller(4),arm_controller(5),arm_controller(6),0.0,0.0;
	Whole_body_controller=control_torques;


};

void Controller::Gripper_controller(double gripper_force){
	/**
	 * This an auxiliary function to determine the finger torques. This will be added to any other torque being commanded during the Return_torques function.
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
		q_des=Robot->finger_position;
		finger_torques+=- (kvj *dq) -kpj*(q-q_des);//add control to keep finger in adecuate position
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

	//if force on gripper is 0, compute open gripper controller
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

//************Base Navigate to a point*****
//state: NAVIGATING
bool navigation_reached_goal(Controller *RobotController , Vector3d point_world){
	Grocery_Robot* Robot=RobotController->Robot;
	return (point_world.head(2)-Robot->q.head(2)).norm()>0.1;
}

bool can_see(Vector3d p1, Vector3d p2){
	double thres = 0.3;
	if(abs(p1(1)-p2(1)) < thres) return true;
	if(abs(p1(0)-p2(0)) < thres){
		for(int i = -1; i < 2; i++){
			if(abs(i*3.65-p1(0)) < thres) return true;
		}
	}
	return false;
}

double L1(Vector3d p1, Vector3d p2){
	return abs(p1(0)-p2(0))+abs(p1(1)-p2(1));
}

void generate_navigation_points_for_object(Grocery_Robot* Robot,Vector3d start, Vector3d goal){
	Robot->navigation_waypoints.col(0) = start;
	Robot->navigation_waypoints.col(1) = goal;
	Robot->navigation_waypoints.col(2) = goal;
	Robot->navigation_waypoints.col(3) = goal;
	Robot->navigation_waypoints.row(2) = VectorXd::Ones(4)*goal(2);
	if(L1(start,goal)<0.1){
		return;
	}
	int pts_need_orientation_cal = 1;
	if(!can_see(start,goal)){
		VectorXd start_sees = VectorXd::Zero(9);
		VectorXd goal_sees = VectorXd::Zero(9);
		for(int i = 0; i < 9; i++){
			if(can_see(start,Robot->aisle_pts.col(i)))start_sees(i) = 1;
			if(can_see(goal,Robot->aisle_pts.col(i)))goal_sees(i) = 1;
			if(start_sees(i)&&goal_sees(i)){
				pts_need_orientation_cal = 2;
				Robot->navigation_waypoints.col(1) = Robot->aisle_pts.col(i);
			}
		}
		if(pts_need_orientation_cal!=2){
			pts_need_orientation_cal = 3;
			double min_dist = 100000;
			for(int i = 0; i < 9; i++){
				for(int j=0; j < 9; j++){
					if(start_sees(i)&&goal_sees(j)){
						Vector3d p1 = Robot->aisle_pts.col(i);
						Vector3d p2 = Robot->aisle_pts.col(j);
						double dist = L1(start,p1)+L1(p1,p2)+L1(p2,goal);
						if(dist < min_dist){
							min_dist = dist;
							Robot->navigation_waypoints.col(1) = p1;
							Robot->navigation_waypoints.col(2) = p2;
						}
					}
				}
			}
		}
	}
	for(int i =0; i < pts_need_orientation_cal; i++){
		double dx = Robot->navigation_waypoints(0,i+1)-Robot->navigation_waypoints(0,i);
		double dy = Robot->navigation_waypoints(1,i+1)-Robot->navigation_waypoints(1,i);
		Robot->navigation_waypoints(2,i)=atan2(dy,dx);
	}
}

VectorXd Navigate_to_point(Controller *RobotController , Vector3d point_world, double V_max){
	/**
	*Function to base navegate to a point, while keeping arm in home position
	* gives the needed torques and perfr=orm other operations needed like log the last platform position.
	**/
	Grocery_Robot* Robot=RobotController->Robot;
	if(navigation_reached_goal(RobotController,point_world)){
		point_world(2) = Robot->q(2);
	}
	RobotController->Base_controller(point_world,V_max);
	//upodate last position of the base
	// Robot->base_position<<Robot->q(0),Robot->q(1),Robot->q(2);
	//send controller
	return RobotController->Return_torques();
}



//************pick_shelf_objects*****

double pick_shelf_objects_counter=0;
Vector3d ref_pos;//used when neede to keep the old position of the object
Vector3d target_pos;
Vector2d pos_front_shelf;//save the initial position in front shelf
VectorXd pick_shelf_objects(Controller *RobotController , Objects_class *Object){
	Grocery_Robot* Robot=RobotController->Robot;
	VectorXd control_torques= VectorXd::Zero(Robot->dof);
	double control_threshold=0.07;
	// double control_threshold=0.9;
	bool arrived = false;
	double V_max=0.4;
	int hold_time=500;//Time to wait when touched the object before start to lift it
	Vector3d x_w;
	Vector3d delta_phi;
	Matrix3d R_des;
	Matrix3d R_w;
	VectorXd force_r;
	VectorXd target_q;
	VectorXd force_l;
	VectorXd force_ee;
	//define horizontal horientation
	R_des=Object->shelf.get_eef_orientation();
	//Todo: align with shelft in other orientations.
	x_w=Robot->x_w;
	switch (Robot->Current_state)
	{

		case MOVING_ARM:
			/**
			 * move arm in front of the shelve.
			 * this will be the heigh of the object, but keeping safe distance from the shelve
			 * this could be also reach using way points
			 * **/

			//targert position a little far away of the object, in front
			target_pos=Object->get_position_front_object_in_shelf();
			pos_front_shelf<<target_pos(0),target_pos(1);
			//move to target
			RobotController->Arm_World_Position_Orientation_controller(target_pos);//control arm and platform, need both to ensure position
			RobotController->Gripper_controller(0);//gripper open
			control_torques=RobotController->Return_torques();

			//if reached postion, move to orient
			if ((target_pos-x_w).norm()<=control_threshold){
				Robot->set_robot_state(Robot_States::ORIENTING_ARM);
			}
			break;
		case ORIENTING_ARM:
			/**
			 * orient horizontal and open grip
			 * /We want the orientation be horizontal to the plane
			 **/
			//targert position a little far away of the object, in front
			target_pos=Object->get_position_front_object_in_shelf();
			//compute error in rotation
			R_w=Robot->R_w;
			delta_phi = -0.5 * (R_w.col(0).cross(R_des.col(0)) + R_w.col(1).cross(R_des.col(1)) + R_w.col(2).cross(R_des.col(2)));
			//move to target
			RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.7);
			RobotController->Gripper_controller(0);
			control_torques=RobotController->Return_torques();

			//if oriented, move to aproacing object
			if (delta_phi.norm()<=1e-1){
				Robot->set_robot_state(Robot_States::APROACHING_OBJECT);
			}

			break;
		case APROACHING_OBJECT:
			/**
			 * aproach object keeping gripper horizontal
			 **/
			target_pos<< Object->x_w[0],Object->x_w[1],Object->x_w[2];
			force_ee=Robot->force_eef;

			//move to target
			RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
			RobotController->Gripper_controller(0);
			control_torques=RobotController->Return_torques();

			//if touch object, stop
			if ((force_ee).norm()>1e-5){//as soon it senses object, go next
				Robot->set_robot_state(Robot_States::PICKING_OBJECT);
				//save as reference current pos
				target_pos<< x_w[0],x_w[1],x_w[2];
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
				if (pick_shelf_objects_counter>=hold_time){
					//forces to hold moving a little up
					RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,0.1);
					RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
					control_torques=RobotController->Return_torques();


					// RobotController->Arm_Local_Position_Orientation_controller(target_pos,Robot->get_base_rotation().transpose()*R_des,0.1);//base frame
					// RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
					// control_torques=RobotController->Return_torques();

					//if reached position
					if ((target_pos-Robot->x_w).norm()<=0.02){
						Robot->set_robot_state(Robot_States::MOVING_BACKWARDS);
						target_pos(0)=pos_front_shelf(0);//give target for moving backwards. do not change z
						target_pos(1)=pos_front_shelf(1);
						target_pos(3)=Robot->x_w(2);
						pick_shelf_objects_counter=0;//reset counter
						cout<<"target poss leaving "<<target_pos.transpose()<<endl;
					}
				}else{
					pick_shelf_objects_counter+=1;
					//cout<< "counter "<< pick_shelf_objects_counter <<endl;
					RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,V_max);
					RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
					control_torques=RobotController->Return_torques();
					//in the last count, add new target position
					if(pick_shelf_objects_counter==hold_time){
						//move a little up
						//target_pos<<Robot->x(0),Robot->x(1),Robot->x(2);//base frame
						target_pos(2)+=0.05;//this is the distance that will go up, latter replace depending on the shelf.
						// Robot->base_position<<Robot->q(0),Robot->q(1),Robot->q(2);
						cout<<"target poss counter "<<target_pos.transpose()<<endl;
					}
				}
			}else{
				//hold for a moment in current position holding the objetc
				RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,V_max);
				RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
				control_torques=RobotController->Return_torques();
				Robot->finger_position<<Robot->q(10),Robot->q(11);
			}
			break;
		case MOVING_BACKWARDS:
			/**
			 * Move backward to leave shelf space.
			 **/
			//target position set in the previous step
			//force
			cout<<"target back "<<target_pos.transpose()<<endl;
			cout<<"pos back "<<Robot->x_w.transpose()<<endl;
			RobotController->Arm_World_Position_Orientation_controller(target_pos,R_des,V_max);
			RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
			control_torques=RobotController->Return_torques();

			if ((target_pos-x_w).norm()<=1e-1){
				Robot->set_robot_state(Robot_States::PLACING_OBJECT);

			}

			break;
		case PLACING_OBJECT:
			/**
			 * placing object in the basket. Follow waypoints in base frame
			 **/
			if (!arrived){//this way, once is true, keep being true
				arrived=Robot->checkWaypoints(Robot->home_to_basket_waypoints, 2, 1);
			}
			target_pos = Robot->next_waypoint_ee;
			cout<<"target"<<target_pos.transpose()<<endl;
			cout<<"current"<<Robot->x.transpose()<<endl;
			//if((target_pos-Robot->x).norm()<=control_threshold){
			if(arrived && (target_pos-Robot->x).norm()<=control_threshold){
					Robot->set_robot_state(Robot_States::R_IDLE);
					control_torques.setZero();
			}else{
				RobotController->Arm_Local_Position_Orientation_controller(target_pos,Robot->Rot_h_ee_basket(),V_max);
				RobotController->Gripper_controller(Robot->pick_up_force);
				control_torques=RobotController->Return_torques();
			}


			// arrived=Robot->checkWaypoints(Robot->home_to_basket_waypoints, 3, 1);
			// target_q = Robot->next_waypoint_ee;
			// if(arrived){
			// 		// Robot->set_robot_state(Robot_States::R_IDLE);
			// 		// control_torques.setZero();
			// }else{
			// 		RobotController->Arm_Joint_controller(target_q);
			// 		RobotController->Gripper_controller(Robot->pick_up_force);//gripper will hold in the designated force
			// 		control_torques=RobotController->Return_torques();
			// }



			break;
		default:
		//default case
			control_torques.setZero();
			Robot->set_robot_state(Robot_States::R_IDLE);
			break;
		}
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
	auto object = new Sai2Model::Sai2Model(object_file, false);
	auto object2 = new Sai2Model::Sai2Model(object2_file, false);
	auto object3 = new Sai2Model::Sai2Model(object3_file, false);
	robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
	object->_q =redis_client.getEigenMatrixJSON(OBJECT1_JOINT_ANGLES_KEY);
	object->_dq =redis_client.getEigenMatrixJSON(OBJECT1_JOINT_VELOCITIES_KEY);
	object2->_q =redis_client.getEigenMatrixJSON(OBJECT2_JOINT_ANGLES_KEY);
	object2->_dq =redis_client.getEigenMatrixJSON(OBJECT2_JOINT_VELOCITIES_KEY);
	object3->_q =redis_client.getEigenMatrixJSON(OBJECT3_JOINT_ANGLES_KEY);
	object3->_dq =redis_client.getEigenMatrixJSON(OBJECT3_JOINT_VELOCITIES_KEY);
	robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
	robot->updateModel();
	object->updateModel();
	object2->updateModel();
	object3->updateModel();




	//define variables
	VectorXd control_torques = VectorXd::Zero(robot->dof());
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
	//Robot.HomePosition<<0.0,0.0,0.0, 0.798855 ,-0.328792, -0.001426 ,-0.006871, -0.000757, -0.053838 ,-0.000491 ,-0.039092,- -0.119215;
	Robot.HomePosition<<0.0,0.0,0, -1.57079632679, -0.785, 0, -2.356, 0, 1.571, 0.785 ,0.04, -0.04;
	Robot.Update_states();
	// Robot.base_position<<Robot.q(0),Robot.q(1),Robot.q(2);

  //aisle attributes
	const VectorXd aisle_xs = (VectorXd(9)<<-3.65,0.0,3.65,-3.65,0.0,3.65,-3.65,0.0,3.65).finished();
	const VectorXd aisle_ys = (VectorXd(9)<<-2.55,-2.55,-2.55,0.0,0.0,0.0,2.55,2.55,2.55).finished();
	Robot.aisle_pts = MatrixXd::Zero(3,9);
	Robot.aisle_pts.row(0) = aisle_xs;
	Robot.aisle_pts.row(1) = aisle_ys;

	//create robot object
	Robot.cur_time = timer.elapsedTime();
	//initialize waypoints
	Robot.navigation_waypoints = MatrixXd(3,4);
	// Robot.navigation_waypoints<<0.0,0.0,0.0,
	// 0.0,0.0,0.0,
	// 0.0,0.0,0.0;
	// Robot.navigation_waypoints.transposeInPlace();

	//Robot.home_to_basket_waypoints = MatrixXd(7,3);
	Robot.home_to_basket_waypoints = MatrixXd(3,3);//IN ROBOT FRAME
	Robot.waypoint_iterator = -1;
	Robot.set_robot_state(Robot_States::R_IDLE);
	//start robot controller
	Controller RobotController;
	RobotController.Robot=&Robot;
	//start with 0 torques
	RobotController.finger_torques=VectorXd::Zero(2);
	RobotController.Whole_body_controller=VectorXd::Zero(robot->dof());


	//create SHELFS. fill with info from worlf urdf
	Shelf Shelf2;
	Shelf2.origin_xyz<<2.35, -0.8, 0.0;
	Shelf2.origin_rpy<<0 ,0 , 0;
	Shelf2.depth=0.463;

	Shelf Shelf8;
	Shelf8.origin_xyz<<-2.35 ,-1.75, 0.0;
	Shelf8.origin_rpy<<0 ,0 , 3.14159265358;
	Shelf8.depth=0.463;

	Shelf Shelf13;
	Shelf13.origin_xyz<<-1.3 ,0.8 ,0.0;
	Shelf13.origin_rpy<<0 ,0 , 3.14159265358;
	Shelf13.depth=0.463;



	//Start OBJECT1 JAR1
	Objects_class Object1;
	Object1.name="jar1";
	Object1.SAI2_object=object;
	Object1.height=0.128;
	Object1.width=0.0888;
	Object1.depth=0.0888;
	Object1.obj_offset<<2.35 ,-0.9 ,0.7+Object1.height/2;
	Object1.shelf=Shelf2;

	//Start OBJECT2 JAR2
	Objects_class Object2;
	Object2.name="jar2";
	Object2.SAI2_object=object2;
	Object2.height=0.128;
	Object2.width=0.0888;
	Object2.depth=0.0888;
	Object2.obj_offset<<-1.3, 0.8 ,1.0+Object2.height/2;
	Object2.shelf=Shelf13;

	//Start OBJECT3 JAR3
	Objects_class Object3;
	Object3.name="jar3";
	Object3.SAI2_object=object3;
	Object3.height=0.128;
	Object3.width=0.0888;
	Object3.depth=0.0888;
	Object3.obj_offset<<-2.35, -1.65, 0.49+Object3.height/2;
	Object3.shelf=Shelf8;



	//list of objects to pickup in the program ("shopping list")
	std::list<Objects_class*> list_objects;
	list_objects.push_back(&Object2);
	list_objects.push_back(&Object3);
	list_objects.push_back(&Object1);


	//retrieve first element in the list
	Objects_class* current_object=list_objects.front();
	list_objects.pop_front();//delete from the list





	runloop = true;
	// cout<<Robot.aisle_pts<<endl;
	// VectorXd s1 = (VectorXd(3)<<1.0,0.0,0.0).finished();
	// VectorXd g1 = (VectorXd(3)<<1.0,2.55,0.0).finished();
	// generate_navigation_points_for_object(&Robot,s1,g1);
	Vector3d Position;

	Position=current_object->shelf.position_front_shelf();
	generate_navigation_points_for_object(&Robot,Robot.base_position,Position);//we will need a function that gives all the trayectory... last point is in front of shelf
	while (runloop)
	{
		// cout<<Robot.navigation_waypoints<<endl;
		// cout<<Robot.navigation_waypoints<<endl;

fTimerDidSleep = timer.waitForNextLoop();

		// read robot and object state from redis
		robot->_q = redis_client.getEigenMatrixJSON(JOINT_ANGLES_KEY);
		robot->_dq = redis_client.getEigenMatrixJSON(JOINT_VELOCITIES_KEY);
		object->_q= redis_client.getEigenMatrixJSON(OBJECT1_JOINT_ANGLES_KEY);
		object2->_q= redis_client.getEigenMatrixJSON(OBJECT2_JOINT_ANGLES_KEY);
		object3->_q= redis_client.getEigenMatrixJSON(OBJECT3_JOINT_ANGLES_KEY);
		robot->updateModel();
		object->updateModel();
		object2->updateModel();
		object3->updateModel();
		//update force sensor
		Robot.force_finger_r=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_r);
		Robot.force_finger_l=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_l);
		Robot.force_eef=redis_client.getEigenMatrixJSON(EE_FORCE_KEY_EEF);


		// update robot information (position orientation)
		Robot.Update_states();
		Robot.cur_time = timer.elapsedTime();

		//update objets position and orientation
		Object1.Update_states();
		Object2.Update_states();
		Object3.Update_states();




		//neede variables for state machine
		Vector3d Position;
		double Nav_Vmax;
		bool arrived=false;
		Vector3d target_pos;//debug







		//State machine
		switch(current_simulation_state)
		{
		case Simulation_states::IDLE:
			/**
			 * Fo nothing state, for the end of simulation
			 **/
			control_torques.setZero();
			break;
		// case Simulation_states::BETWEEN_WAYPOINTS:
		// 	break;
		case Simulation_states::GO_TO_SHELF:
			/**
			 * aproach robot platform in front of the shelf
			 * move platform with arm in home position.
			 **/
			//Define waypoints to go to shelf
			//last point is shelf position (x,y,theta)

			//position fron of shelf x,y,z
			// generate_navigation_points_for_object(&Robot,Robot.base_position,Position);//we will need a function that gives all the trayectory... last point is in front of shelf
			//add needed rotation
			// Robot.navigation_waypoints.col(0)<<0,0,0;
			// Robot.navigation_waypoints.col(1)<<Position(0),Position(1),0;

			// cout<<"goal"<<Robot.next_waypoint_base,transpose()<<endl;
			// cout<<"current"<<Robot.q(0)<<" "<<Robot.q(1)<<" "<<Robot.q(2)<<endl;
			arrived=Robot.checkWaypoints(Robot.navigation_waypoints, 4, 0);
			control_torques=Navigate_to_point(&RobotController,Robot.next_waypoint_base,0.7);
			if(arrived){
				//go next state, pick from shelf
				Robot.set_robot_state(Robot_States::MOVING_ARM);
				//set_simulation_state(Simulation_states::PICK_SHELF_OBJECTS);//skipt for test
				//test lines:
				current_object=list_objects.front();
				list_objects.pop_front();//delete from the list
				set_simulation_state(Simulation_states::GO_TO_SHELF);//go to the next shelf
				Position=current_object->shelf.position_front_shelf();
				generate_navigation_points_for_object(&Robot,Robot.base_position,Position);
				//end test lines
				control_torques.setZero();
			}






			break;
		case Simulation_states::PICK_SHELF_OBJECTS:
			switch (Robot.Current_state)
			{
			//if robot is idle, go to the next object in the list
			//if list ended, move to go to conveyor
			 case Robot_States::R_IDLE:
				//check if there are more element in the list, if not, move go to conveyor
				if(list_objects.size()==0){
					set_simulation_state(Simulation_states::GO_TO_CONVEYOR);
					control_torques.setZero();
				}else{//else update current object
					current_object=list_objects.front();
					list_objects.pop_front();//delete from the list
					set_simulation_state(Simulation_states::GO_TO_SHELF);//go to the next shelf
					Position=current_object->shelf.position_front_shelf();
					generate_navigation_points_for_object(&Robot,Robot.base_position,Position);
					Robot.set_robot_state(Robot_States::R_IDLE);
					control_torques=pick_shelf_objects(&RobotController,current_object);
				}
				break;
			 default:
			    //default, continue in the picking shelf function
				control_torques=pick_shelf_objects(&RobotController,current_object);
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
