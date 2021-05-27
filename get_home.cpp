#include "get_home.h"

CHoming::CHoming(int jdof, const int jointtype[])
{
    _joint_num = jdof;
    _count = 0;
    _init_time_bool = false;
    _operation_bool = false;
    _print_start_bool = false;
    _init_t = 0.0;
    _start_t = 0.0;
    _wait_t = 5.0;
    _motion_t = 3.5;
    _time_count = 0.0;
    _motion_range_rad_max.setZero(jdof);
    _motion_range_rad_max.setConstant(10.0*DEG2RAD);// +- 10deg
    _motion_range_rad_min.setZero(jdof);
    _motion_range_rad_min.setConstant(-10.0*DEG2RAD);// +- 10deg
    _motion_range_m_max.setZero(jdof);
    _motion_range_m_max.setConstant(0.1);// +- 0.1m for prismatic joint
    _motion_range_m_min.setZero(jdof);
    _motion_range_m_min.setConstant(-0.1);// +- 0.1m for prismatic joint
    _qdot_des.setZero(jdof);
    _q_offset.setZero(jdof);
    _t_motion_1.setZero(jdof);
    _t_motion_2.setZero(jdof);
    _t_motion_3.setZero(jdof);
    _motion_range_max.setZero(jdof);
    _motion_range_min.setZero(jdof);
    _joint_type.setZero(jdof);

    _q.setZero(jdof);
    _home_sensor.setZero(jdof);
    _home_sensor_pre.setZero(jdof);

    _q_start_touch.setZero(jdof);
    _q_end_touch.setZero(jdof);

    _bool_start_touch.resize(jdof);
    _bool_start_touch.setConstant(false);
    _bool_end_touch.resize(jdof);
    _bool_end_touch.setConstant(false);
    
     _t_motion_1.setConstant(_motion_t);
     _t_motion_2.setConstant(_motion_t*2.0);
     _t_motion_3.setConstant(_motion_t);
    

    int arr_low[] = {2, 4, 5, 6, 9, 11, 12, 13};

    for(int i = 0; i < 8; i++ )
    {
        _motion_t = 2.0;
        _t_motion_1(arr_low[i]) = _motion_t;
        _t_motion_2(arr_low[i]) = _motion_t* 2.0;
        _t_motion_3(arr_low[i]) = _motion_t;
    }

    _tot_motion_t = 0.0;
    for(int i=0; i<_joint_num; i++)
    {
        _tot_motion_t = _tot_motion_t + (_t_motion_1(i) + _t_motion_2(i) + _t_motion_3(i));
    }
    cout << "Total motion time: " << _tot_motion_t << "sec" <<endl;
    

    _motion_range_rad_max(0) = 5.0*DEG2RAD;
    _motion_range_rad_min(0) = -30.0*DEG2RAD;
    _motion_range_rad_max(1) = 80.0*DEG2RAD;
    _motion_range_rad_min(1) = -10.0*DEG2RAD;
    _motion_range_rad_max(2) =  10.0*DEG2RAD;
    _motion_range_rad_min(2) = -30.0*DEG2RAD;
    _motion_range_rad_max(3) = 45.0*DEG2RAD;
    _motion_range_rad_min(3) = -5.0*DEG2RAD;
    _motion_range_rad_max(4) = 10.0*DEG2RAD;
    _motion_range_rad_min(4) = -30.0*DEG2RAD;
    _motion_range_rad_max(5) = 10.0*DEG2RAD;
    _motion_range_rad_min(5) = -20.0*DEG2RAD;
    _motion_range_rad_max(6) = 15.0*DEG2RAD;
    _motion_range_rad_min(6) = -20.0*DEG2RAD;
    _motion_range_rad_max(7) = -5.0*DEG2RAD;
    _motion_range_rad_min(7) = 30.0*DEG2RAD;
    _motion_range_rad_max(8) = -80.0*DEG2RAD;
    _motion_range_rad_min(8) = 10.0*DEG2RAD;
    _motion_range_rad_max(9) = 30.0*DEG2RAD;
    _motion_range_rad_min(9) = -10.0*DEG2RAD;
    _motion_range_rad_max(10) = -45.0*DEG2RAD;
    _motion_range_rad_min(10) = 5.0*DEG2RAD;
    _motion_range_rad_max(11) = 30.0*DEG2RAD;
    _motion_range_rad_min(11) = -10.0*DEG2RAD;
    _motion_range_rad_max(12) = -10.0*DEG2RAD;
    _motion_range_rad_min(12) = 20.0*DEG2RAD;
    _motion_range_rad_max(13) = 20.0*DEG2RAD;
    _motion_range_rad_min(13) = -15.0*DEG2RAD;
    _motion_range_rad_max(14) = 0.0;
    _motion_range_rad_min(14) = 0.0;

    for(int i=0; i<jdof; i++)
    {
        _joint_type(i) = jointtype[i];

        if(jointtype[i] == 0) //revolute joint
        {
            _motion_range_max(i) = _motion_range_rad_max(i);
            _motion_range_min(i) = _motion_range_rad_min(i);
        }
        else if(jointtype[i] == 1) //prismatic joint
        {
            _motion_range_max(i) = _motion_range_m_max(i);
            _motion_range_min(i) = _motion_range_m_min(i);
        }
    }
}

CHoming::~CHoming()
{

}

void CHoming::read(double time, double joint_position[], int home_sensor[])
{
    if(_init_time_bool == false)
    {
        _init_t = time;
        _start_t = _init_t + _wait_t;
        _init_time_bool = true;
    }
    _t = time;

    for(int i=0; i<_joint_num; i++)
    {
        _q(i) = joint_position[i];        
        _home_sensor_pre(i) = _home_sensor(i);        

        if(home_sensor[i] == 0)
        {
            _home_sensor(i) = 0;
        }
        else
        {
            _home_sensor(i) = 1;
        }        
    }    
}

void CHoming::write(double desired_velocity[], double offset_position[])
{
    for(int i=0; i<_joint_num; i++)
    {
        desired_velocity[i] = _qdot_des(i);
        offset_position[i] = _q_offset(i);
    }

    //TODO: add output, position offset
}

void CHoming::joint_velocity_homing_motion()
{    
    if(_t < _start_t && _operation_bool == false)
    {
        _qdot_des.setZero();
    }
    //else if(_t >= _start_t && _t <=_start_t + 4.0*_motion_t*_joint_num && _operation_bool == false)
    else if(_t >= _start_t && _t <=_start_t + _tot_motion_t && _operation_bool == false)    
    {
        if(_print_start_bool == false)
        {
            cout <<endl<<endl<< "Homing motion start, time: " << _t << "sec."<<endl<<endl;
            _print_start_bool = true;
        }
        _qdot_des.setZero();

        if(_t >= _start_t +_time_count  && _t < _start_t+_t_motion_1(_count) +_time_count)
        {
            _qdot_des.setZero();
            _qdot_des(_count) = CustomMath::CubicDot(_t,_start_t+_time_count, _start_t+_t_motion_1(_count) +_time_count, 0.0, 0.0, _motion_range_max(_count),0.0);            
        }
        else if(_t >= _start_t+_t_motion_1(_count) +_time_count && _t < _start_t+_t_motion_1(_count)+_t_motion_2(_count) +_time_count)
        {
            _qdot_des.setZero();
            _qdot_des(_count) = CustomMath::CubicDot(_t,_start_t+_t_motion_1(_count) +_time_count, _start_t+_t_motion_1(_count)+_t_motion_2(_count) +_time_count, 0.0, 0.0, -_motion_range_max(_count) + _motion_range_min(_count),0.0);            
            
            check_home_sensor();
        }
        else if(_t >= _start_t+_t_motion_1(_count)+_t_motion_2(_count) +_time_count && _t < _start_t+_t_motion_1(_count)+_t_motion_2(_count)+_t_motion_3(_count) +_time_count)
        {
            _qdot_des.setZero();
            _qdot_des(_count) = CustomMath::CubicDot(_t,_start_t+_t_motion_1(_count)+_t_motion_2(_count) +_time_count, _start_t+_t_motion_1(_count)+_t_motion_2(_count)+_t_motion_3(_count) +_time_count, 0.0, 0.0, -_motion_range_min(_count),0.0);            
        }
        else if(_t >= _start_t+_t_motion_1(_count)+_t_motion_2(_count)+_t_motion_3(_count) +_time_count)
        {
            cout << "Homing motion for joint " << _count << " completed, time: " << _t << "sec."<<endl<<endl;
            _time_count = _time_count + _t_motion_1(_count) + _t_motion_2(_count) + _t_motion_3(_count);
            _count = _count + 1;            
            _qdot_des.setZero();            
        }        
    }
    //else if(_t >_start_t + 4.0*_motion_t*_joint_num && _operation_bool == false)
    else if(_t >_start_t + _tot_motion_t && _operation_bool == false)
    {
        cout << "Homing motion for joint " << _count << " completed, time: " << _t << "sec."<<endl<<endl;
        calculate_homeoffset();
        cout << "Homing Completed!" <<endl;
        _qdot_des.setZero();
        _operation_bool = true;
    }

    else
    {
        _qdot_des.setZero();
    }
}

void CHoming::check_home_sensor()
{    
    for(int i=0; i<_joint_num; i++)
    {
        if(_home_sensor(i) == 0 && _home_sensor_pre(i) == 1) //0 -> 1
        {
            _q_start_touch(i) = _q(i);
            _bool_start_touch(i) = true;
        }
        else if(_home_sensor(i) == 1 && _home_sensor_pre(i) == 0) //1 -> 0
        {
            _q_end_touch(i) = _q(i);
            _bool_end_touch(i) = true;
        }
    }
}
void CHoming::calculate_homeoffset()
{
    for(int i=0; i<_joint_num; i++)
    {
        if(_bool_start_touch(i) == false || _bool_end_touch(i) == false)
        {
            cout << "Homing failed: Joint "<< i <<endl;
            _q_offset(i) = 0.0;
        }
        else
        {
            _q_offset(i) = (_q_start_touch(i) + _q_end_touch(i))/2.0;
            cout << "Detected Home Position of Joint "<< i << ": " << _q_offset(i) << endl;
        }
    }
}

void CHoming::reset_count()
{
    _time_count = 0.0;
    _count = 0;
    _operation_bool = false;
    _print_start_bool = false;
}

void CHoming::homing_velocity_control() //this must be run after 'read' and before 'write'
{
    joint_velocity_homing_motion();
}