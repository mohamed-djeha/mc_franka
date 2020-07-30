#pragma once

#include <mc_rbdyn/Robot.h>

#include <franka/exception.h>
#include <franka/robot.h>

#include "ControlMode.h"

template<ControlMode cm>
struct PandaControlType
{
  static_assert(static_cast<int>(cm) == static_cast<int>(cm) + 1, "This must be specialized");
};

template<>
struct PandaControlType<ControlMode::Position> : public franka::JointPositions
{
  using ReturnT = franka::JointPositions;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointPositions(state.q), prev_q_(state.q) {}

  // Interpolate control value from the data in a robot
  franka::JointPositions update( mc_rbdyn::Robot & robot, rbd::MultiBodyConfig & mbc, size_t iter, size_t N, const std::array<double, 7> & coriolis, const Eigen::Matrix<double, 7, 1> & massTorque, const std::array<double, 7> & tauMeasure, const std::array<double, 7> & integral, const double & torqueInput, const double & posGain, const double & velGain, double & feedback, const double & period, const std::array<double, 7> & qEncoder, const std::array<double, 7> & dqEncoder)
  { //integral += M_PI / 30 * 0.001;
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < q.size(); ++i)
    { // if (period > 0.001)
      //{
      //  //double qIn = robot.encoderValues()[i] + robot.encoderVelocities()[i] * period + 0.5 * period * period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  //double alphaIn = robot.encoderVelocities()[i] + period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  //double qIn = qEncoder[i] + dqEncoder[i] * period + 0.5 * period * period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  //double alphaIn = dqEncoder[i] + period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  double qIn = mbc.q[robot.jointIndexByName(rjo[i])][0] + mbc.alpha[robot.jointIndexByName(rjo[i])][0] * period + 0.5 * period * period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  double alphaIn = mbc.alpha[robot.jointIndexByName(rjo[i])][0] + period * mbc.alphaD[robot.jointIndexByName(rjo[i])][0];
      //  robot.mbc().q[robot.jointIndexByName(rjo[i])][0] = qIn;
      //  robot.mbc().alpha[robot.jointIndexByName(rjo[i])][0] = alphaIn ;
      //  q[i] = qIn;
      //}
      //else
      //{
      //  q[i] = mbc.q[robot.jointIndexByName(rjo[i])][0];
      //}
      //if (i == 1)
      //{
        q[i] = integral[i];  
      //}
      //else
      //{
       // q[i] = mbc.q[robot.jointIndexByName(rjo[i])][0];
      //}
      //q[i] = prev_q_[i] + (iter + 1) * (mbc.q[robot.jointIndexByName(rjo[i])][0] - prev_q_[i]) / N;
      //
    }
    if(iter + 1 == N)
    {
      prev_q_ = q;
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 100); 
  }

private:
  std::array<double, 7> prev_q_;
};

template<>
struct PandaControlType<ControlMode::Velocity> : public franka::JointVelocities
{
  using ReturnT = franka::JointVelocities;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::JointVelocities(state.dq) {}

  // Update control value from the data in a robot
  franka::JointVelocities update(const mc_rbdyn::Robot & robot, const rbd::MultiBodyConfig & mbc, size_t, size_t, const std::array<double, 7> & coriolis, const Eigen::Matrix<double, 7, 1> & massTorque, const std::array<double, 7> & tauMeasure, const std::array<double, 7> & integral, const double & torqueInput, const double & posGain, const double & velGain, double & feedback, const double & period, const std::array<double, 7> &, const std::array<double, 7> &)
  {
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < dq.size(); ++i)
    {
      dq[i] = mbc.alpha[robot.jointIndexByName(rjo[i])][0];
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, franka::ControllerMode::kJointImpedance, true, 1000);
  }
};

template<>
struct PandaControlType<ControlMode::Torque> : public franka::Torques
{
  using ReturnT = franka::Torques;
  using CallbackT = std::function<ReturnT(const franka::RobotState &, franka::Duration)>;

  PandaControlType(const franka::RobotState & state) : franka::Torques(state.tau_J) {}

  // Update control value from the data in a robot
  franka::Torques update(const mc_rbdyn::Robot & robot,  const rbd::MultiBodyConfig & mbc, size_t, size_t, const std::array<double, 7> & coriolis, const Eigen::Matrix<double, 7, 1> & massTorque, const std::array<double, 7> & tauMeasure, const std::array<double, 7> & integral, double & torqueInput, const double & posGain, double & velGain, double & feedback, const double & period, const std::array<double, 7> &, const std::array<double, 7> &)
  { 
    //LOG_INFO("flag k: "<< integral);
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < tau_J.size(); ++i)
    { //mbc.jointTorque[robot.jointIndexByName(rjo[i])][0] -= gravity[i];
      //tau_J[i] = mbc.jointTorque[robot.jointIndexByName(rjo[i])][0] - gravity[i];
      //torqueInput = coriolis[i] + massTorque(i,0);// + velGain * (mbc.alpha[robot.jointIndexByName(rjo[i])][0] - robot.encoderVelocities()[i]);
     // if (i == 6)
//     // {
//     //   double P = 0.5;
//     //   double I = 0.01;
//     //   double slideTorque = 0.32;
//     //   double absVel = std::abs(robot.encoderVelocities()[i]);
//     //   double e =  slideTorque - tauMeasure[i];
//     //   integral += e;
//     //   tau_J[i] = torqueInput;
//     //   //if (tauMeasure[i] < 0.32)
//     //   //{
//     //   //  feedback = P * e + I * integral;
//     //   //  tau_J[i] = torqueInput + feedback;
//     //   //}
//     //   //else
//     //   //{
//     //   //  tau_J[i] = torqueInput + feedback;
//     //   //}
//     //   //if ((absVel < 0.005) && (torqueInput < 0.01))
//     //   //{
//     //   //  double e = slideTorque - tauMeasure[i];
//     //   //  integral += e;
//     //   //  tau_J[i] = torqueInput + P * e + I * integral;
//     //   //}
//     //   //else if ((absVel < 0.005) && (torqueInput > -0.01))
//     //   //{
//     //   //  double e = -slideTorque - tauMeasure[i];
//     //   //  integral += e;
//
     //     //tau_J[i] = torqueInput + P * e + I * integral;
     //   //}
     //   //else
     //   //{
     //   //  tau_J[i] = torqueInput;// + velGain * (mbc.alpha[robot.jointIndexByName(rjo[i])][0] - robot.encoderVelocities()[i]);
     //   //}
     // }
     // else
     // {
     //   tau_J[i] =  0;//torqueInput + velGain * (mbc.alpha[robot.jointIndexByName(rjo[i])][0] - robot.encoderVelocities()[i]);
     // }
     // velGain = 2 * std::sqrt(posGain);
     // tau_J[i] = coriolis[i] + massTorque(i,0) + posGain * (mbc.q[robot.jointIndexByName(rjo[i])][0] - robot.encoderValues()[i]) +  velGain * (mbc.alpha[robot.jointIndexByName(rjo[i])][0] - robot.encoderVelocities()[i]);
      tau_J[i] = coriolis[i] + massTorque(i,0) + velGain * (mbc.alpha[robot.jointIndexByName(rjo[i])][0] - robot.encoderVelocities()[i]);
      //tau_J[i] = mbc.jointTorque[robot.jointIndexByName(rjo[i])][0];
    }
    return *this;
  }

  void control(franka::Robot & robot, CallbackT cb)
  {
    robot.control(cb, true, 1000);
  }
};
