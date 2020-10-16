#include "Sai2Model.h"
#include "redis/RedisClient.h"
#include "timer/LoopTimer.h"
#include <math.h>

#include <iostream>
#include <string>
#include <fstream>
#include <complex>

#define QUESTION_1   1
#define QUESTION_2   2
#define QUESTION_3   3
#define QUESTION_4   4

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
	const Vector3d pos_in_link = Vector3d(0, 0, 0.15);
	VectorXd command_torques = VectorXd::Zero(dof);

	// model quantities for operational space control
	MatrixXd Jv = MatrixXd::Zero(3,dof);
	MatrixXd Lambda = MatrixXd::Zero(3,3);
	MatrixXd J_bar = MatrixXd::Zero(dof,3);
	MatrixXd N = MatrixXd::Zero(dof,dof);

	robot->Jv(Jv, link_name, pos_in_link);
	robot->taskInertiaMatrix(Lambda, Jv);
	robot->dynConsistentInverseJacobian(J_bar, Jv);
	robot->nullspaceMatrix(N, Jv);

	// create a timer
	LoopTimer timer;
	timer.initializeTimer();
	timer.setLoopFrequency(1000);
	double start_time = timer.elapsedTime(); //secs
	bool fTimerDidSleep = true;

	std::ofstream c1, c2, c3, c4;
	c1.open("../../hw3/controller1.txt");
	c2.open("../../hw3/controller2.txt");
	c3.open("../../hw3/controller3.txt");
	c4.open("../../hw3/controller4.txt");

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
		int controller_number = QUESTION_3;

		// ---------------------------  question 1 ---------------------------------------
		if(controller_number == QUESTION_1)
		{
			VectorXd g(dof);
			robot->gravityVector(g);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			Vector3d xd;
			xd << 0.3 + 0.1*sin(M_PI*time), 0.1 + 0.1*cos(M_PI*time), 0.5;
			Vector3d xd_d;
			xd_d << 0.1*M_PI*cos(M_PI*time), -0.1*M_PI*sin(M_PI*time), 0.0;
			Vector3d xd_dd;
			xd_dd << -0.1*pow(M_PI,2.0)*sin(M_PI*time), -0.1*pow(M_PI,2.0)*cos(M_PI*time), 0.0;


			Vector3d pos;
			robot->position(pos, link_name, pos_in_link);

			Vector3d vel;
			robot->linearVelocity(vel, link_name, pos_in_link);

			VectorXd qd = VectorXd::Zero(dof);
			qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			double kp = 100.0;
			double kv = 20.0;
			double kpj = 50.0;
			double kvj = 14.0;

			c1  << pos(0) << "," << pos(1) << "," << pos(2) << \
			"," << xd(0) << "," << xd(1) << "," << xd(2) << "\n";

			VectorXd force = VectorXd::Zero(dof);
			//force = Lambda*(-kp*(pos - xd) - kv*vel);
			force = Lambda*(xd_dd -kp*(pos - xd) - kv*(vel-xd_d));
			command_torques << Jv.transpose()*force + g - N.transpose()*(kpj*(robot->_q - qd) + kvj*robot->_dq);
		}

		// ---------------------------  question 2 ---------------------------------------
		if(controller_number == QUESTION_2)
		{
			VectorXd g(dof);
			robot->gravityVector(g);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			Vector3d xd;
			//xd << -0.1, 0.15, 0.2; //Question d
			xd << -0.65, -0.45, 0.7;

			Vector3d pos;
			robot->position(pos, link_name, pos_in_link);

			Vector3d vel;
			robot->linearVelocity(vel, link_name, pos_in_link);

			VectorXd qd = VectorXd::Zero(dof);
			qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			VectorXd q_up = VectorXd::Zero(dof);
			q_up << -M_PI*165.0/180.0, -M_PI*100.0/180.0, -M_PI*165.0/180.0, \
			-M_PI*170.0/180.0, -M_PI*165.0/180.0, 0.0, -M_PI*165.0/180.0;

			VectorXd q_low = VectorXd::Zero(dof);
			q_low << M_PI*165.0/180.0, M_PI*100.0/180.0, M_PI*165.0/180.0, \
			-M_PI*30.0/180.0, M_PI*165.0/180.0, M_PI*210.0/180.0, M_PI*165.0/180.0;

			double kp = 100.0;
			double kv = 20.0;
			double kdamp = 14.0;
			double kmid = 25.0;

			c2  << pos(0) << "," << pos(1) << "," << pos(2) << \
			"," << xd(0) << "," << xd(1) << "," << xd(2) << "," << \
			robot->_q(3) << "," <<  robot->_q(5) << ",""\n";

			VectorXd force = VectorXd::Zero(dof);
			force = Lambda*(-kp*(pos - xd) - kv*(vel));
			//command_torques << Jv.transpose()*force + g - N.transpose()*(kdamp*robot->_dq); //Question d
			//command_torques << Jv.transpose()*force + g - N.transpose()*(kdamp*robot->_dq + 2*kmid*(robot->_q - 0.5*(q_up + q_low)));
			command_torques << Jv.transpose()*force + g - 2*kmid*(robot->_q - 0.5*(q_up + q_low)) - N.transpose()*(kdamp*robot->_dq);
		}

		// ---------------------------  question 3 ---------------------------------------
		if(controller_number == QUESTION_3)
		{

			VectorXd g(dof);
			robot->gravityVector(g);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			Vector3d xd;
			xd << 0.6, 0.3, 0.5;

			Vector3d pos;
			robot->position(pos, link_name, pos_in_link);

			Vector3d vel;
			robot->linearVelocity(vel, link_name, pos_in_link);

			Matrix3d rot;
			robot->rotation(rot, link_name);

			Matrix3d rd = Matrix3d::Zero(3,3);;
			rd << cos(M_PI/3), 0.0, sin(M_PI/3),
						0.0, 1.0, 0.0,
						-sin(M_PI/3), 0.0, cos(M_PI/3);

			Vector3d delta_phi;
			Sai2Model::orientationError(delta_phi, rd, rot);

			Vector3d avel;
			robot->angularVelocity(avel, link_name, pos_in_link);

			double kp = 100.0;
			double kv = 18.0;
			double kvj = 10.0;

			c3  << pos(0) << "," << pos(1) << "," << pos(2) << \
			"," << xd(0) << "," << xd(1) << "," << xd(2) << "," << \
			delta_phi(0) << "," << delta_phi(1) << "," << delta_phi(2) << "\n";

			VectorXd V = VectorXd::Zero(6);
			V.block<3, 1>(0, 0) << kp*(xd - pos) - kv*vel;
			V.block<3, 1>(3, 0) << kp*(- delta_phi) - kv*avel;

			//cout << kp*(- delta_phi) - kv*avel << endl;

			MatrixXd J;
			robot->J_0(J, link_name, pos_in_link);

			MatrixXd Lambda0;
			robot->taskInertiaMatrix(Lambda0, J);

			VectorXd force = VectorXd::Zero(6);
			force = Lambda0*V;
			command_torques << J.transpose()*force + g - N.transpose()*(kvj*robot->_dq);

		}

		// ---------------------------  question 4 ---------------------------------------
		if(controller_number == QUESTION_4)
		{

			VectorXd g(dof);
			robot->gravityVector(g);

			robot->Jv(Jv, link_name, pos_in_link);
			robot->taskInertiaMatrix(Lambda, Jv);
			robot->dynConsistentInverseJacobian(J_bar, Jv);
			robot->nullspaceMatrix(N, Jv);

			//robot->operationalSpaceMatrices(Lambda, J_bar, N, Jv);

			Vector3d x_desired;
			x_desired << 0.6, 0.3, 0.4;

			Vector3d pos;
			robot->position(pos, link_name, pos_in_link);

			Vector3d vel;
			robot->linearVelocity(vel, link_name, pos_in_link);

			VectorXd qd = VectorXd::Zero(dof);
			qd << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

			double kp = 200.0;
			double kv = 28.0;

			double kvj = 30.0;
			double kpj = 100.0;

			Vector3d vel_desired;
			vel_desired << kp/kv *(x_desired - pos);

			double Vmax = 0.1;


			MatrixXd S = MatrixXd::Zero(3,3);

			for (int i=0; i<3; i++){
				double v = Vmax/abs(vel_desired(i));
				if (abs(v)<=1.0)
				{
					S(i,i) = v;
				}
				else
				{
					S(i,i) = (v > 0) - (v < 0);

				}
			}


			c4  << pos(0) << "," << pos(1) << "," << pos(2) << "," << \
	 		x_desired(0) << "," << x_desired(1) << "," << x_desired(2) << \
			"," << vel(0) << "," << vel(1) << "," << vel(2) << "," << Vmax <<  "\n";

			VectorXd force = VectorXd::Zero(dof);
			force = Lambda*(-kp*(pos - x_desired) - kv*(vel));
			//force = Lambda*(- kv*(vel - S*vel_desired));
			command_torques << Jv.transpose()*force + g - N.transpose()*robot->_M*(kpj*(robot->_q - qd) + kvj*robot->_dq);
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
