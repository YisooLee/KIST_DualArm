#pragma once
#ifndef __TASKCONTROLLER_H
#define __TASKCONTROLLER_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include "custommath.h"

#define CustomArraySize 50
namespace Eigen
{
		typedef Matrix<double, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXd;
		typedef Matrix<double, -1, -1, 0, CustomArraySize, 1> VectorCXd;
        typedef Matrix<int, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXi;
		typedef Matrix<int, -1, -1, 0, CustomArraySize, 1> VectorCXi;
        typedef Matrix<bool, -1, -1, 0, CustomArraySize, CustomArraySize> MatrixCXb;
		typedef Matrix<bool, -1, -1, 0, CustomArraySize, 1> VectorCXb;
}

using namespace std;
using namespace Eigen;

class CTaskController
{
    public:
	CTaskController(int jdof);
	virtual ~CTaskController();

    void read();
    void write();
    void compute();
    
    int _jdof;

    private:
    void model_update();

};

#endif