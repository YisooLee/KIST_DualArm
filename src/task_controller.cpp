#include "task_controller.h"

CTaskController::CTaskController(int jdof)
{
    _jdof = jdof;
    initialize();
}

CTaskController::~CTaskController()
{

}


void CTaskController::read(double time, double joint_position[], double joint_velocity[], double joint_force[])
{
    _time = time;

    // Note: order of joint of model and elmo direver is different
    _q(0) = joint_position[14];
    _qdot(0) = joint_velocity[14];
    _torque(0) = joint_force[14];

    for(int i=0; i<_jdof-1; i++)
    {
        _q(i+1) = joint_position[i];
        _qdot(i+1) = joint_velocity[i];
        _torque(i+1) = joint_force[i];
    }

    //TODO: qdot filtered will be added
}

void CTaskController::model_update()
{
    //RBDL will be added
}

void CTaskController::write(double torque_desired[])
{
    // Note: order of joint of model and elmo direver is different    
    for(int i=0; i<_jdof-1; i++)
    {
        torque_desired[i] = _torque_des(i+1);
    }
    torque_desired[14] = _torque_des(0);
}

void CTaskController::compute()
{
    _torque_des.setZero();
    //_torque_des(1) = -10.0;
    //_torque_des(8) = 10.0;
}


void CTaskController::initialize()
{
    _time = 0.0;

    _q.setZero(_jdof);
    _qdot.setZero(_jdof);
    _torque.setZero(_jdof);

    _q_des.setZero(_jdof);
    _qdot_des.setZero(_jdof);
    _torque_des.setZero(_jdof);
}