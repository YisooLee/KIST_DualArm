#pragma once
#ifndef __TASKCONTROLLER_H
#define __TASKCONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"
#include "robotmodel.h"

using namespace std;
using namespace Eigen;

class CTaskController
{
    public:
	CTaskController(int jdof);
	virtual ~CTaskController();

    void read(double time, double joint_position[], double joint_velocity[], double joint_force[]);
    void write(double torque_desired[]);
    void compute();
    
    int _jdof;

    private:
    void initialize();
    void model_update();

    double _time;
    VectorCXd _q, _qdot, _torque; //state
    VectorCXd _q_des, _qdot_des, _torque_des; //reference
    
    CModel Model; //robot model
};

#endif