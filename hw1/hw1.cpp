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

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	std::ofstream c1, c2, c3, c4, c5;
	c1.open("../../hw1/controller1.txt");
  c2.open("../../hw1/controller2.txt");
	c3.open("../../hw1/controller3.txt");
	c4.open("../../hw1/controller4.txt");
	c5.open("../../hw1/controller5.txt");

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
		int controller_number = QUESTION_4;  // change to the controller of the question you want : QUESTION_1, QUESTION_2, QUESTION_3, QUESTION_4, QUESTION_5


		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			double kp = 400.0;      // chose your p gain
			double kv = 48.0;      // chose your d gain

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired << M_PI/2, -M_PI/4, 0.0, -125*M_PI/180, 0, 80*M_PI/180, 0;
			c1  << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) << "\n" ;

			command_torques.setZero();  // change to the control torques you compute
			command_torques << -kp*(robot->_q - q_desired) - kv*robot->_dq;
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			int dof = robot->dof();
			VectorXd g(dof);
			robot->gravityVector(g);
			double kp = 400.0;      // chose your p gain
			double kv = 48.0;      // chose your d gain

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired << M_PI/2, -M_PI/4, 0.0, -125*M_PI/180, 0, 80*M_PI/180, 0;
			c2  << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) << "\n" ;

			command_torques.setZero();  // change to the control torques you compute
			command_torques << -kp*(robot->_q - q_desired) - kv*robot->_dq +  g;
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{
			int dof = robot->dof();
			VectorXd g(dof);
			robot->gravityVector(g);

			//std::string ee_link_name = "link7";
			//Eigen::MatrixXd ee_jacobian(3,dof);
		  //robot->Jv(ee_jacobian,ee_link_name);

			double kp = 400.0;      // chose your p gain
			double kv = 36.0;      // chose your d gain

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired << M_PI/2, -M_PI/4, 0.0, -125*M_PI/180, 0, 80*M_PI/180, 0;
			c3  << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) << "\n" ;

			command_torques.setZero();  // change to the control torques you compute
			//cout << robot->_M.block(0,6, 7,1) << endl;
			//cout << robot->_M << endl;
			command_torques << robot->_M*(-kp*(robot->_q - q_desired) - kv*robot->_dq) +  g;
		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			int dof = robot->dof();
			VectorXd g(dof);
			robot->gravityVector(g);

			std::string ee_link_name = "link7";
			Eigen::MatrixXd ee_jacobian(3,dof);
		  robot->Jv(ee_jacobian,ee_link_name);

			VectorXd b(dof);
			robot->coriolisForce(b);

			double kp = 400.0;      // chose your p gain
			double kv = 40.0;      // chose your d gain

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired << M_PI/2, -M_PI/4, 0.0, -125*M_PI/180, 0, 80*M_PI/180, 0;
			c4  << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) << "\n" ;

			command_torques.setZero();  // change to the control torques you compute
			command_torques << robot->_M*(-kp*(robot->_q - q_desired) - kv*robot->_dq) +  g + b;

		}

		// ---------------------------  question 5 ---------------------------------------
		if(controller_number == QUESTION_5)
		{

			int dof = robot->dof();
			VectorXd g(dof);
			robot->gravityVector(g);

			std::string ee_link_name = "link7";
			Eigen::MatrixXd ee_jacobian(3,dof);
		  robot->Jv(ee_jacobian,ee_link_name);

			VectorXd b(dof);
			robot->coriolisForce(b);

			double kp = 400.0;      // chose your p gain
			double kv = 34.0;      // chose your d gain

			//cout << robot->_M.block(6,0, 1,7) << endl;

			VectorXd q_desired = initial_q;   // change to the desired robot joint angles for the question
			q_desired << M_PI/2, -M_PI/4, 0.0, -125*M_PI/180, 0, 80*M_PI/180, 0;
			c5  << robot->_q(0) << "," << robot->_q(2) << "," << robot->_q(3) << "\n" ;

			command_torques.setZero();  // change to the control torques you compute
			command_torques << robot->_M*(-kp*(robot->_q - q_desired) - kv*robot->_dq) +  g + b;
		}

		// **********************
		// WRITE YOUR CODE BEFORE
		// **********************

		// send to redis
		redis_client.setEigenMatrixJSON(JOINT_TORQUES_COMMANDED_KEY, command_torques);

		controller_counter++;

	}

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
