#ifndef CONTROLVARIABLE_HPP_
#define CONTROLVARIABLE_HPP_

#include <array>

namespace control_variable
{
    // size must be multiply of 4 for a memory alignement.
    // EtherCAT -> PC
    struct ControlVariable
    {
        double wheel_vel_FL;
        double wheel_vel_FR;
        double wheel_vel_RL;
        double wheel_vel_RR;

        double steering_vel_FL;
        double steering_vel_FR;
        double steering_vel_RL;
        double steering_vel_RR;

        double steering_pos_FL;
        double steering_pos_FR;
        double steering_pos_RL;
        double steering_pos_RR;

        double lift_vel_FL;
        double lift_vel_FR;
        double lift_vel_RL;
        double lift_vel_RR;

        double lift_pos_FL;
        double lift_pos_FR;
        double lift_pos_RL;
        double lift_pos_RR;

        double lift_trq_FL;
        double lift_trq_FR;
        double lift_trq_RL;
        double lift_trq_RR;

        void reset()
        {
            wheel_vel_FL = 0.0;
            wheel_vel_FR = 0.0;
            wheel_vel_RL = 0.0;
            wheel_vel_RR = 0.0;

            steering_vel_FL = 0.0;
            steering_vel_FR = 0.0;
            steering_vel_RL = 0.0;
            steering_vel_RR = 0.0;

            steering_pos_FL = 0.0;
            steering_pos_FR = 0.0;
            steering_pos_RL = 0.0;
            steering_pos_RR = 0.0;

            lift_vel_FL = 0.0;
            lift_vel_FR = 0.0;
            lift_vel_RL = 0.0;
            lift_vel_RR = 0.0;

            lift_pos_FL = 0.0;
            lift_pos_FR = 0.0;
            lift_pos_RL = 0.0;
            lift_pos_RR = 0.0;

            lift_trq_FL = 0.0;
            lift_trq_FR = 0.0;
            lift_trq_RL = 0.0;
            lift_trq_RR = 0.0;
        }
    };
}
#endif // CONTROLVARIABLE_HPP_