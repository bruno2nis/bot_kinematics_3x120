#ifndef DEF_KINEMATICS_H
#define DEF_KINEMATICS_H

/** Class to managed robot kinematics.
 *
 *  Used for setup geometric parameter of a bot and for calculate inverse
 *  and direct kinematics functions. 
 *
 * This class is intended for robots which move with three wheels with 
 * concurrent axes at a point named C for center, 
 * angularly uniformly distributed every 120° and 
 * all all at an equal distance from point C. 
 * 
 * The wheels are designated in an orderly fashion; the first is at the back 
 * of the robot, the second at the front right and the last at the front left.
 * The speed of the robot's wheels (called joint speed in robotics) 
 * is expressed in radians per second. 
 *
 * The speed of the robot (called end-effector speed in robotics) is define as 
 * a triplet veltangent (forward speed), velnormal (lateral speed to the left) 
 * and velangular (speed of rotation around the vertical axis passing through 
 * the point C). For veltangent and velnormal the unit is the meter per second
 *  while for velangular the unit is the radian per second.
 *  
 * Example:
 * @code
 * #include "mbed.h"
 * #include "kinematics.h"
 * 
 * kinematics kine; // create a instance of kinematics without parameters
 *                  // kinematics kine (0.019, 0.08) // create kine with param
 * 
 * // end-effector speed
 * float veltangent; // forward speed of bot (m/s)
 * float velnormal; // lateral speed to left of bot (m/s)
 * float velangular; // vertical rotational speed of bot around point C (rad/s)
 * 
 * // Joint speeds
 * float omega_wheel_1; // angular speed of rear wheel (rad/s)
 * float omega_wheel_2; // angular speed of front right wheel (rad/s)
 * float omega_wheel_3; // angular speed of front left wheel (rad/s)
 * 
 * // vector to store wheel speeds
 * std::vector< float > omega;
 * 
 * int main() {
 * 
 *     // setup geometric parametters
 *     kine.set_wheel_radius(0.019);
 *     kine.set_wheel_distance_to_bot_center(0.080);
 *     
 *     // Set robot speed
 *     veltangent = 0.5; // m/s
 *     velnormal = 0.0;  // m/s
 *     velangular = 0.0; // rad/s
 * 
 *     // calculate joint speed from robot speed
 *     omega = kine.inverse(veltangent, velnormal, velangular);
 * 
 *     // dispatch return vector into scalar variables
 *     omega_wheel_1 = omega[0]; // rad/s
 *     omega_wheel_2 = omega[1]; // rad/s
 *     omega_wheel_3 = omega[3]; // rad/s
 * }
 * @endcode
 */

#include <vector>

class kinematics
{
    public:
    
        /** Create kinematics instance 
         *
         * @param wheel_radius Wheel radius (unit: meter)
         * @param wheel_distance_to_bot_center Distance from the center of 
         *        robot to the center of wheels (unit: meter)
         */
        kinematics(float wheel_radius = 0.019,
                   float wheel_distance_to_bot_center = 0.080);

        /** Set radius of wheels
         *
         * @param radius Wheel radius (unit: meter)
         */    
        void set_wheel_radius(float radius);
        
        /** Get radius of wheels
         *
         * @return radius Wheel radius (unit: meter)
         */    
        float get_wheel_radius();
        
        /** Set distance from center of wheels en center of robot 
         *
         * @param radius Distance from the center of 
         *        robot to the center of wheels (unit: meter)
         */ 
        void set_wheel_distance_to_bot_center(float radius);
        
        /** Get distance from center of wheels en center of robot 
         *
         * @returns radius Distance from the center of 
         *          robot to the center of wheels (unit: meter)
         */ 
        float get_wheel_distance_to_bot_center();

        /** Calculate angular velocity of wheels from robot velocity
         *
         * @param veltangent tangent velocity of the robot (forward speed,
         *        unit: m/s)
         *
         * @param velnormal normal velocity of robot (left speed, unit: m/s)
         *
         * @param velangular angular velocity of robot (rotation around
         *        vertical axis, unit: rad/s)
         *
         * @returns
         *  A three dimensional vector of float:  
         *    index 0 the rotational speed of wheel #1 (unit: rad/s),
         *    index 1 the rotational speed of wheel #2 (unit: rad/s),
         *    index 2 the rotational speed of wheel #3 (unit: rad/s),
         */
        std::vector<float> inverse(float veltangent, 
                                   float velnormal, 
                                   float velangular);
    
        //void set_wheel_distance_to_bot_center(float radius);
        
        /** Calculate the velocity of robot from wheel velocities
         *
         * @param omega1 the rotational speed of wheel #1 (unit: rad/s)
         *
         * @param omega2 the rotational speed of wheel #2 (unit: rad/s)
         *
         * @param omega3 the rotational speed of wheel #3 (unit: rad/s)
         *
         * @returns
         *  A three dimensional vector of float:  
         *    index 0 the tangent velocity of the robot 
         *            (forward speed, unit: m/s),
         *    index 1 the normal velocity of robot (left speed, unit: m/s),
         *    index 2 the angular velocity of robot (rotation around
         *            vertical axis, unit: rad/s)
         */
        std::vector<float> direct(float omega1, float omega2, float omega3);
    
    private:
    
    /* Geometric data */

    /* Wheel radius (m) */
    float _wr;
    
    /* Distance from the center of robot to the center of wheels (m) */
    float _w2c; // distance from wheel to bot center
    
    /* Constante data to avoid multiple recalculation.
     * Values shared between all instances of the class (static) 
     */

    /* sqrt3_2 store square root of three divide by 2 */
    static const float _sqrt3_2 = 0.866025;
    
    /* sqrt3 store square root of three */
    static const float _sqrt3 = 1.73205;
    
    };

#endif /* DEF_KINEMATICS_H */



