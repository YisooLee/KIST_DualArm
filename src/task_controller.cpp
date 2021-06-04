#include "task_controller.h"

CTaskController::CTaskController(int jdof, double dt, const double touch_probe_position_rad[])
{    
    _jdof = jdof;
    _dt = dt;

    initialize();

    _q_touch_probe(0) = touch_probe_position_rad[14];
    for(int i=0; i<_jdof-1; i++)
    {
        _q_touch_probe(i+1) = touch_probe_position_rad[i];
    }
    
}

CTaskController::~CTaskController()
{

}

void CTaskController::get_joint_position_offset(double joint_position_offset[])
{
    _q_offset(0) = joint_position_offset[14];

    for(int i=0; i<_jdof-1; i++)
    {
        _q_offset(i+1) = joint_position_offset[i];
    }
}

void CTaskController::read(double time, double joint_position[], double joint_velocity[], double joint_force[])
{
    _time = time;

    // Note: order of joint of model and elmo direver is different
    _q(0) = joint_position[14] - (_q_offset(0) - _q_touch_probe(0));
    _qdot(0) = joint_velocity[14];
    _torque(0) = joint_force[14];

    for(int i=0; i<_jdof-1; i++)
    {
        _q(i+1) = joint_position[i] -(_q_offset(i+1) - _q_touch_probe(i+1));
        _qdot(i+1) = joint_velocity[i];
        _torque(i+1) = joint_force[i];
    }

    //TODO: qdot filtered will be added
}

void CTaskController::model_update()
{	
    Model.update_kinematics(_q, _qdot);
	Model.update_dynamics();
	Model.calculate_EE_Jacobians();
	Model.calculate_EE_positions_orientations();
	Model.calculate_EE_velocity();

	//set Jacobian
	_J_hands.block<6, 15>(0, 0) = Model._J_left_hand;
	_J_hands.block<6, 15>(6, 0) = Model._J_right_hand;
	_J_T_hands = _J_hands.transpose();

	_J_ori_hands.block<3, 15>(0, 0) = Model._J_left_hand_ori;
	_J_ori_hands.block<3, 15>(3, 0) = Model._J_right_hand_ori;
	_J_ori_T_hands = _J_ori_hands.transpose();
	_J_pos_hands.block<3, 15>(0, 0) = Model._J_left_hand_pos;
	_J_pos_hands.block<3, 15>(3, 0) = Model._J_right_hand_pos;
	_J_pos_T_hands = _J_pos_hands.transpose();

	//calc Jacobian dot (with lowpass filter)	
	for (int i = 0; i < 12; i++)
	{
		for (int j = 0; j < 15; j++)
		{
			_Jdot_hands(i,j) = CustomMath::VelLowpassFilter(_dt, 2.0 * PI * 20.0, _pre_J_hands(i,j), _J_hands(i,j), _pre_Jdot_hands(i,j)); //low-pass filter

			_pre_J_hands(i, j) = _J_hands(i, j);
			_pre_Jdot_hands(i, j) = _Jdot_hands(i, j);
		}		
	}
	_Jdot_qdot.noalias() = _Jdot_hands * _qdot;
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
    model_update();

    _torque_des.setZero();
    //_torque_des(1) = -10.0;
    //_torque_des(8) = 10.0;

    //cout << _q.transpose() <<endl;
    cout << Model._bg.transpose() <<endl;
    cout <<"q " << _q.transpose() <<endl;
    cout <<"qdot " << _qdot.transpose() <<endl;
    cout <<"lh " << Model._x_left_hand.transpose() <<endl;
    cout <<"rh " << Model._x_right_hand.transpose() <<endl;
}


void CTaskController::initialize()
{
    _time = 0.0;

    _q.setZero(_jdof);
    _qdot.setZero(_jdof);
    _torque.setZero(_jdof);

    _q_offset.setZero(_jdof);
    _q_touch_probe.setZero(_jdof);

    _q_des.setZero(_jdof);
    _qdot_des.setZero(_jdof);
    _torque_des.setZero(_jdof);

    _J_hands.setZero(12, _jdof);
    _J_T_hands.setZero(_jdof, 12);
	_Jdot_hands.setZero(12, _jdof);	
    _pre_J_hands.setZero(12, _jdof);
	_pre_Jdot_hands.setZero(12, _jdof);
    _Jdot_qdot.setZero(12);

    _J_pos_hands.setZero(6, _jdof);
	_J_pos_T_hands.setZero(_jdof, 6);
	_J_ori_hands.setZero(6, _jdof);
	_J_ori_T_hands.setZero(_jdof, 6);
	
}