# include "kinematics.h"

kinematics::kinematics(float wheel_radius, 
                       float wheel_distance_to_bot_center)
{
    /* Wheel radius (m) */
    _wr = wheel_radius;

    /* Distance from the center of robot to the center of wheels (m) */
    _w2c = wheel_distance_to_bot_center;
}

void kinematics::set_wheel_radius(float radius)
{
    /* Wheel radius (m) */
    _wr = radius;
}

float kinematics::get_wheel_radius()
{
    /* Wheel radius (m) */
    return _wr;
}

void kinematics::set_wheel_distance_to_bot_center(float radius)
{
    /* Distance from the center of robot to the center of wheels (m) */
    _w2c = radius;
}

float kinematics::get_wheel_distance_to_bot_center()
{
    /* Distance from the center of robot to the center of wheels (m) */
    return _w2c;
}

std::vector<float> kinematics::inverse(float veltangent, 
                                       float velnormal, 
                                       float velangular)
{
    std::vector<float> omega(3);
    
    omega[0] = (                         - velnormal + _w2c*velangular) / _wr;
    omega[1] = ( _sqrt3_2*veltangent + 0.5*velnormal + _w2c*velangular) / _wr;
    omega[2] = (          veltangent + 0.5*velnormal + _w2c*velangular) / _wr;

    return omega;
}

std::vector<float> kinematics::direct( float omega1, float omega2, float omega3)
{
    std::vector<float> velocity(3);
    
    /* tangential velocity */
    velocity[0] = _wr * (            omega2/_sqrt3 - omega3/_sqrt3);
    velocity[1] = _wr * (-2*omega1 + omega2        + omega3       ) / 3;
    velocity[2] = _wr * (   omega1 + omega2        + omega3       ) / (3*_w2c);
    
    return velocity;
}
