#include <mc_control/mc_global_controller.h>

#include <boost/program_options.hpp>
namespace po = boost::program_options;
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <chrono>
#include <condition_variable>
#include <iostream>
#include <thread>
#include <mc_rtc/logging.h>
namespace mc_time
{

using duration_us = std::chrono::duration<double, std::micro>;
/** Always pick a steady clock */
using clock = typename std::conditional<std::chrono::high_resolution_clock::is_steady,
                                        std::chrono::high_resolution_clock,
                                        std::chrono::steady_clock>::type;

} // namespace mc_time

#include "PandaControlType.h"

std::condition_variable sensors_cv;
std::mutex sensors_mutex;
std::condition_variable command_cv;
std::mutex command_mutex;

template<ControlMode cm>
struct PandaControlLoop
{
  PandaControlLoop(const std::string & name, const std::string & ip, size_t steps)
  : name(name), robot(ip), state(robot.readOnce()), control(state), steps(steps)
  { 
  }

  void updateSensors(mc_rbdyn::Robot & robot, mc_rbdyn::Robot & real)
  {
    using get_sensor_t = const std::vector<double> & (mc_rbdyn::Robot::*)() const;
    using set_sensor_t = void (mc_rbdyn::Robot::*)(const std::vector<double> &);
    auto updateSensor = [&](get_sensor_t get, set_sensor_t set, const std::array<double, 7> & value) {
      auto sensor = (robot.*get)();
      if(sensor.size() != robot.refJointOrder().size())
      {
        sensor.resize(robot.refJointOrder().size());
      }
      for(size_t i = 0; i < value.size(); ++i)
      {
        sensor[i] = value[i];
      }
      (robot.*set)(sensor);
      (real.*set)(sensor);
    };
    updateSensor(&mc_rbdyn::Robot::encoderValues, &mc_rbdyn::Robot::encoderValues, state.q);
    updateSensor(&mc_rbdyn::Robot::encoderVelocities, &mc_rbdyn::Robot::encoderVelocities, state.dq);
    updateSensor(&mc_rbdyn::Robot::jointTorques, &mc_rbdyn::Robot::jointTorques, state.tau_J);
    command = robot.mbc();
  }

  void updateSensors(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);

    std::vector<double> positionFranka;
    std::vector<double> velocityFranka;
    std::vector<double> torqueFranka;
    positionFranka.reserve(7);
    velocityFranka.reserve(7);
    torqueFranka.reserve(7);
    positionFranka.assign(std::begin(state.q),std::end(state.q));
    velocityFranka.assign(std::begin(state.dq),std::end(state.dq));
    torqueFranka.assign(std::begin(state.tau_J),std::end(state.tau_J));
    controller.setEncoderValues(positionFranka);
    controller.setEncoderVelocities(velocityFranka);
    controller.setJointTorques(torqueFranka);

    updateSensors(robot, real);
  }

  void init(mc_control::MCGlobalController & controller)
  {
    auto & robot = controller.controller().robots().robot(name);
    auto & real = controller.controller().realRobots().robot(name);
    updateSensors(robot, real);
    const auto & rjo = robot.refJointOrder();
    for(size_t i = 0; i < rjo.size(); ++i)
    {
      auto jIndex = robot.jointIndexByName(rjo[i]);
      robot.mbc().q[jIndex][0] = state.q[i];
      robot.mbc().jointTorque[jIndex][0] = state.tau_J[i];
    }
    robot.forwardKinematics();
    real.mbc() = robot.mbc();
    integral_init = state.q;
    integral_q = integral_init[index];
    integral_dq = 0.;
    integral = integral_init;
    //e_prec = state.q[4] - q_goal;
  }

  void control_thread(mc_control::MCGlobalController & controller)
  { 
    control.control(robot,
                    [ this, &controller ](const franka::RobotState & stateIn, franka::Duration time) ->
                    typename PandaControlType<cm>::ReturnT {
                      this->state = stateIn;
                      sensor_id += 1;
                      auto & robot = controller.controller().robots().robot(name);
                      auto & real = controller.controller().realRobots().robot(name);
                      
                      if(sensor_id % steps == 0)
                      {
                        sensors_cv.notify_all();
                        std::unique_lock<std::mutex> command_lock(command_mutex);
                        command_cv.wait(command_lock);
                      }
                      period = time.toSec();
                      //t += 0.001;
                      //dt = 0.001;
                      t += period;
                      dt = period;

                      if (period == 0)
                      {
                        Init = state.q[index];
                      }
                      //double delta_angle = M_PI / 30. * t;
                      //double delta_angle = M_PI / 8.0 * (1 - std::cos(M_PI / 5 * t)) + Init;
                      double delta_angle = - std::sin(M_PI / 10 * t);// + integral_init[index]; // for square signal
                     // q_goal = delta_angle;
                      // generate a square signal
                      if (delta_angle <= 0)//integral_init[index])
                      {
                        q_goal = 0;// + integral_init[index];
                      }
                      else
                      {
                        q_goal = 1;// + integral_init[index]; 
                      }

                      //double acc = 0.1;
                      
                      D = 2 * std::sqrt(P);
                      
                      double e = state.q[index] - q_goal;
                      if (std::abs(e) < 1e-2)
                      { 
                        e = 0;
                        //integral_q = state.q[index];
                        //integral_dq = state.dq[index];
                        acc = - D * state.dq[index] - D * integral_dq  - P * (e);
                      }
                      else
                      {
                        acc = - D * state.dq[index] - P * (e);
                      }

                      // de = 0; 

                      //if(dt != 0)
                      //{  de = (e - e_prec) / dt;
                      //}
        
                      
                      e_prec = e;
                      
                      //acc = - D * state.dq[index] - P * (e);
                      //double acc = - D * de - P * e;
                      integral_q += integral_dq * dt + 0.5 * acc * dt * dt;
                      integral_dq += acc * dt;
                     // integral = {{integral_init[0], integral_init[1] ,
                     //              integral_init[2], integral_init[3],
                     //              integral_init[4] + delta_angle, integral_init[5], integral_init[6]}};
                      integral[index] = integral_q;
                     // integral[index] = q_goal;
                     // integral = {{integral_init[0], integral_init[1] ,
                     //              integral_init[2], integral_init[3],
                     //              state.q[4] + 0.005, integral_init[5], integral_init[6]}};
                     // integral = {{integral_init[0], integral_init[1] ,
                     //              integral_init[2], integral_init[3],
                     //              state.q_d[4] + state.dq_d[4] * 0.001 + 0.5 * 0.00001 * 5, integral_init[5], integral_init[6]}};
                     // integral_q = state.q + state.dq * 0.001 + 0.5 * 0.00001 * 0.1; 
                      //integral_dq = 
                      //------------------------------------
                        controller.controller().logger().addLogEntry("qRef", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(integral[j]);
                          }
                          return ret;
                        });
                        //------------------------------------
                        controller.controller().logger().addLogEntry("alphaRef", [this]() {
                          double ret = integral_dq;
                          return ret;
                        });
                        //------------------------------------
                      //  controller.controller().gui()->addElement(
                      //    {"Franka"}, mc_rtc::gui::NumberInput("qGoal", [this]() { return q_goal; }, [this](double d) { q_goal = d; }));
                      //  //------------------------------------
                        controller.controller().logger().addLogEntry("qGoal", [this]() {
                          double ret = q_goal;
                          return ret;
                        });

                      if(controller.running)
                      { 
                        gravity = model.gravity(state);
                        coriolis = model.coriolis(state);
                        massMatrix_array = model.mass(state);
                        //tauMeasure = state.tau_J;
                        Eigen::Map<const Eigen::Matrix<double, 7, 7>> massMatrix(massMatrix_array.data());
                        //Eigen::Map<const Eigen::Matrix<double, 7, 1>> accelerationFranka(state.ddq_d.data());
                        
                        const auto & rjo = robot.refJointOrder();
                        int i = 0;
                        for(const auto & j : rjo)
                        {
                          accelerationQP(i, 0) = robot.mbc().alphaD[robot.jointIndexByName(j)][0];
                          i += 1;
                        }
                         
                        //massTorque.reserve(7);
                        //massTorque = massMatrix * accelerationFranka;
                       massTorque = massMatrix * accelerationQP;

                        //LOG_INFO("----------")
                        //LOG_INFO("gravity[0]" << gravity[0]);
                        //LOG_INFO("gravity[1]" << gravity[1]);
                        //LOG_INFO("gravity[2]" << gravity[2]);
                        //LOG_INFO("gravity[3]" << gravity[3]);
                        //LOG_INFO("gravity[4]" << gravity[4]);
                        //LOG_INFO("gravity[5]" << gravity[5]);
                        //LOG_INFO("gravity[6]" << gravity[6]);
                        //LOG_INFO("----------");
                        //LOG_INFO("massTorque[0]" << massTorque(0,0));
                        //LOG_INFO("massTorque[1]" << massTorque(1,0));
                        //LOG_INFO("massTorque[3]" << massTorque(2,0));
                        //LOG_INFO("massTorque[4]" << massTorque(3,0));
                        //LOG_INFO("massTorque[5]" << massTorque(4,0));
                        //LOG_INFO("massTorque[6]" << massTorque(5,0));
                        //LOG_INFO("massTorque[7]" << massTorque(6,0));
                        //------------------------------------
                        controller.controller().logger().addLogEntry("gravity", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(gravity[j]);
                          }
                          return ret;
                        });
                        //------------------------------------
                        controller.controller().logger().addLogEntry("gravity&coriolis", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(gravity[j] + coriolis[j]);
                          }
                          return ret;
                        });
                        //------------------------------------
                          controller.controller().logger().addLogEntry("q_d", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.q_d[j]);
                          }
                          return ret;
                        });
                          //------------------------------------
                          controller.controller().logger().addLogEntry("Error", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(q_goal - state.q[j]);
                          }
                          return ret;
                        });
                          //------------------------------------
                          controller.controller().logger().addLogEntry("alpha_d", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.dq_d[j]);
                          }
                          return ret;
                        });
                         //------------------------------------tau_ext_hat_filtered
                          controller.controller().logger().addLogEntry("tauDesired", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.tau_J_d[j]);
                          }
                          return ret;
                        });
                          //------------------------------------
                          controller.controller().logger().addLogEntry("tauExt", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.tau_ext_hat_filtered[j]);
                          }
                          return ret;
                        });
                          //------------------------------------
                       //   controller.controller().logger().addLogEntry("tauMeasured", [this]() {
                       //   std::vector<double> ret;
                       //   ret.reserve(7);
                       //   for(size_t j = 0; j < 7; ++j)
                       //   {
                       //     ret.push_back(state.tau_J[j]);
                       //   }
                       //   return ret;
                       // });
                       //   //------------------------------------
                       //   controller.controller().logger().addLogEntry("dTauFranka", [this]() {
                       //   std::vector<double> ret;
                       //   ret.reserve(7);
                       //   for(size_t j = 0; j < 7; ++j)
                       //   {
                       //     ret.push_back(state.dtau_J[j]);
                       //   }
                       //   return ret;
                       // });
                        //------------------------------------
                       //  controller.controller().logger().addLogEntry("qFranka", [this]() {
                       //  std::vector<double> ret;
                       //  ret.reserve(7);
                       //  for(size_t j = 0; j < 7; ++j)
                       //  {
                       //    ret.push_back(state.q[j]);
                       //  }
                       //  return ret;
                       // });
                         //------------------------------------
                         controller.controller().logger().addLogEntry("alphaDFranka", [this]() {
                         std::vector<double> ret;
                         ret.reserve(7);
                         for(size_t j = 0; j < 7; ++j)
                         {
                           ret.push_back(state.ddq_d[j]);
                         }
                         return ret;
                        });
                         //------------------------------------
                         controller.controller().logger().addLogEntry("massTorque", [this]() {
                         std::vector<double> ret;
                         ret.reserve(7);
                         for(size_t j = 0; j < 7; ++j)
                         {
                           ret.push_back( massTorque(j,0));
                         }
                         return ret;
                        });
                        //------------------------------------
                         controller.controller().logger().addLogEntry("accQP", [this]() {
                         std::vector<double> ret;
                         ret.reserve(7);
                         for(size_t j = 0; j < 7; ++j)
                         {
                           ret.push_back( accelerationQP(j,0));
                         }
                         return ret;
                        });
                        //------------------------------------
                        controller.controller().logger().addLogEntry("mass&gravity&coriolis", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(gravity[j] + coriolis[j] + massTorque(j,0));
                          }
                          return ret;
                        });
                       // //------------------------------------
                        controller.controller().logger().addLogEntry("TauIn-Gravity", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.tau_J[j] - gravity[j]);
                          }
                          return ret;
                        });
                       // //------------------------------------
                        controller.controller().logger().addLogEntry("dTheta", [this]() {
                          std::vector<double> ret;
                          ret.reserve(7);
                          for(size_t j = 0; j < 7; ++j)
                          {
                            ret.push_back(state.dtheta[j]);
                          }
                          return ret;
                        });
                       // //------------------------------------
                        controller.controller().logger().addLogEntry("ControlSuccessRate", [this]() {
                          double ret = state.control_command_success_rate;
                          return ret;
                        });
                         //------------------------------------
                        controller.controller().logger().addLogEntry("VelocityFeedbackGain", [this]() {
                          double ret = velGain;
                          return ret;
                        });
                         //------------------------------------
                        controller.controller().logger().addLogEntry("torqueStep", [this]() {
                          double ret = torqueStep;
                          return ret;
                        });
                        //------------------------------------
                        controller.controller().logger().addLogEntry("timeFranka", [this]() {
                          double ret = period * 1000;
                          return ret;
                        });
                        //------------------------------------
                        controller.controller().logger().addLogEntry("de", [this]() {
                          double ret = de;
                          return ret;
                        });
                        //-------------------------------------
                        controller.controller().logger().addLogEntry("Stiffness", [this]() {
                          double ret = P;
                          return ret;
                        });
                         //------------------------------------
                        controller.controller().logger().addLogEntry("Damping", [this]() {
                          double ret = D;
                          return ret;
                        });
                         //------------------------------------
                        controller.controller().logger().addLogEntry("acc", [this]() {
                          double ret = acc;
                          return ret;
                        });
                         //------------------------------------
                      //  controller.controller().gui()->addElement(
                      //    {"Franka"}, mc_rtc::gui::NumberInput("Torque input", [this]() { return torqueStep; }, [this](double d) { torqueStep = d; }));
                      //   //------------------------------------
                      //  controller.controller().gui()->addElement(
                      //    {"Franka"}, mc_rtc::gui::NumberInput("Velocity feedback gain", [this]() { return velGain; }, [this](double d) { velGain = d; }));
                      //  //------------------------------------
                      //  controller.controller().gui()->addElement(
                      //    {"Franka"}, mc_rtc::gui::NumberInput("Position feedback gain", [this]() { return posGain; }, [this](double d) { posGain = d; }));
                        //------------------------------------
                        controller.controller().gui()->addElement(
                          {"Franka"}, mc_rtc::gui::NumberInput("Stiffness", [this]() { return P; }, [this](double d) { P = d; }));
                        //------------------------------------
                        controller.controller().gui()->addElement(
                          {"Franka"}, mc_rtc::gui::NumberInput("Damping", [this]() { return D; }, [this](double d) { D = d; }));
                        //------------------------------------
                        controller.controller().gui()->addElement(
                          {{"Franka"}, "Target"}, mc_rtc::gui::NumberInput("Panda_jointA5", [this]() { return q_goal; }, [this](double d) { q_goal = d; }));
                        
                        return control.update(robot, command, sensor_id % steps, steps, coriolis, massTorque, state.tau_ext_hat_filtered, integral, torqueStep, posGain, velGain, feedback, period, state.q_d, state.dq_d);
                      }
                      return franka::MotionFinished(control);
                    });
  }

  std::string name;
  franka::Robot robot;
  franka::RobotState state;
  franka::Model model = robot.loadModel();
  std::array<double, 7> gravity = model.gravity(state);
  std::array<double, 7> coriolis = model.coriolis(state);
  std::array<double, 7> tauMeasure = state.tau_J;
  //std::array<double, 7> alphaDFranka = state.ddq_d;
  std::array<double, 49> massMatrix_array = model.mass(state);
  Eigen::Matrix<double, 7, 1> massTorque;
  Eigen::Matrix<double, 7, 1> accelerationQP;
  Eigen::Matrix<double, 7, 7> massMatrix;
  double torqueStep = 0.;
  double velGain = 10.;
  double posGain = 1.;
  double t = 0.;
  double dt;
  double period;
  double integral_q;
  double integral_dq;
  double q_goal = 0;
  double P = 2;
  double D;
  double e_prec;
  double de;
  double acc;
  int index = 6;
  double Init;
  //std::vector<double> massMatrix_vector;
  //massMatrix_vector.reserve(49);
  //massMatrix_vector.assign(std::begin(massMatrix_array), std::end(massMatrix_array));
  //double integral;
  std::array<double, 7> integral;
  std::array<double, 7> integral_init;
  //double integral_init;// = state.q[1];
  double feedback = 0.;
  //Eigen::Map<const Eigen::Matrix<double, 7, 7>> massMatrix(std::array<double, 49> massMatrix_array.data());
  PandaControlType<cm> control;
  size_t sensor_id = 0;
  size_t steps = 1;
  rbd::MultiBodyConfig command;
};

template<ControlMode cm>
void global_thread(mc_control::MCGlobalController::GlobalConfiguration & gconfig)
{
  auto frankaConfig = gconfig.config("Franka");
  auto ignoredRobots = frankaConfig("ignored", std::vector<std::string>{});
  mc_control::MCGlobalController controller(gconfig);
  if(controller.controller().timeStep < 0.001)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("mc_rtc cannot run faster than 1kHz with mc_franka");
  }
  size_t n_steps = std::ceil(controller.controller().timeStep / 0.001);
  size_t freq = std::ceil(1 / controller.controller().timeStep);
  mc_rtc::log::info("mc_rtc running at {}Hz, will interpolate every {} panda control step", freq, n_steps);
  auto & robots = controller.controller().robots();
  // Initialize all real robots
  for(size_t i = controller.realRobots().size(); i < robots.size(); ++i)
  {
    controller.realRobots().robotCopy(robots.robot(i));
    controller.realRobots().robots().back().name(robots.robot(i).name());
  }
  // Initialize controlled panda robot
  std::vector<std::pair<PandaControlLoop<cm>, size_t>> pandas;
  for(auto & robot : robots)
  {
    if(robot.mb().nrDof() == 0)
    {
      continue;
    }
    if(std::find(ignoredRobots.begin(), ignoredRobots.end(), robot.name()) != ignoredRobots.end())
    {
      continue;
    }
    if(frankaConfig.has(robot.name()))
    {
      std::string ip = frankaConfig(robot.name())("ip");
      pandas.emplace_back(std::make_pair<PandaControlLoop<cm>, size_t>({robot.name(), ip, n_steps}, 0));
      pandas.back().first.init(controller);
    }
    else
    {
      mc_rtc::log::warning("The loaded controller uses an actuated robot that is not configured and not ignored: {}",
                           robot.name());
    }
  }
  for(auto & panda : pandas)
  {
    controller.controller().logger().addLogEntry(panda.first.name + "_sensors_id", [&panda]() { return panda.second; });
  }
  controller.init(robots.robot().encoderValues());
  controller.running = true;
  controller.controller().gui()->addElement(
      {"Franka"}, mc_rtc::gui::Button("Stop controller", [&controller]() { controller.running = false; }));
  // Start panda control loops
  std::vector<std::thread> panda_threads;
  for(auto & panda : pandas)
  {
    panda_threads.emplace_back([&panda, &controller]() { panda.first.control_thread(controller); });
  }
  size_t iter = 0;
  while(controller.running)
  {
    {
      std::unique_lock<std::mutex> sensors_lock(sensors_mutex);
      bool start_measure = false;
      std::chrono::time_point<mc_time::clock> start;
      sensors_cv.wait(sensors_lock, [&]() {
        if(!start_measure)
        {
          start_measure = true;
          start = mc_time::clock::now();
        }
        for(const auto & panda : pandas)
        {
          if(panda.first.sensor_id % n_steps != 0 || panda.first.sensor_id == panda.second)
          {
            return false;
          }
        }
        return true;
      });
      if(iter++ % 5 * freq == 0 && pandas.size() > 1)
      {
        mc_time::duration_us delay = mc_time::clock::now() - start;
        mc_rtc::log::info("[mc_franka] Measured delay between the pandas: {}us", delay.count());
      }
      for(auto & panda : pandas)
      {
        panda.first.updateSensors(controller);
        panda.second = panda.first.sensor_id;
      }
      command_cv.notify_all();
    }
    controller.run();
  }
  for(auto & th : panda_threads)
  {
    th.join();
  }
}

template<ControlMode cm>
struct GlobalControlLoop
{
  mc_control::MCGlobalController & controller;
  std::vector<PandaControlLoop<cm>> robots;
};

int main(int argc, char * argv[])
{
  std::string conf_file = "";
  po::options_description desc("MCUDPControl options");
  // clang-format off
  desc.add_options()
    ("help", "Display help message")
    ("conf,f", po::value<std::string>(&conf_file), "Configuration file");
  // clang-format on

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help"))
  {
    std::cout << desc << "\n";
    std::cout << "see etc/sample.yaml for libfranka configuration\n";
    return 1;
  }

  mc_control::MCGlobalController::GlobalConfiguration gconfig(conf_file, nullptr);
  if(!gconfig.config.has("Franka"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "No Franka section in the configuration, see etc/sample.yaml for an example");
  }
  auto frankaConfig = gconfig.config("Franka");
  ControlMode cm = frankaConfig("ControlMode", ControlMode::Velocity);
  try
  {
    switch(cm)
    {
      case ControlMode::Position:
        global_thread<ControlMode::Position>(gconfig);
        break;
      case ControlMode::Velocity:
        global_thread<ControlMode::Velocity>(gconfig);
        break;
      case ControlMode::Torque:
        global_thread<ControlMode::Torque>(gconfig);
        break;
    }
  }
  catch(const franka::Exception & e)
  {
    std::cerr << "franka::Exception " << e.what() << "\n";
    return 1;
  }
  return 0;
}
