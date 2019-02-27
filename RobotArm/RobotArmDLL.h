#pragma once

#ifdef DLLEXPORT
#define ROBOTARMDLL __declspec(dllexport)
#else
#define ROBOTARMDLL __declspec(dllimport)
#endif

typedef void* pRobotArm;

extern "C" ROBOTARMDLL pRobotArm RobotArmConstruct();
extern "C" ROBOTARMDLL void RobotArmInitialization(double epsilon, unsigned int maximum_iteration, double damped_factor, pRobotArm pRobot);
extern "C" ROBOTARMDLL void RobotArmInverseKinematics(double *q_input, double *Xd, double *q_output, double *X_output, pRobotArm pRobot);
