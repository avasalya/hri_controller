#ifndef _H_MIN_JERK_TRAJ_H_
#define _H_MIN_JERK_TRAJ_H_

#include <cmath>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <mc_control/mc_controller.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_rbdyn/robot.h>
#include <mc_rtc/logging.h>
#include <Tasks/QPTasks.h>

// Generated by cmake
#include "datapath.h"

/// min-jerk-trajectory namespace ///
namespace mj_traj
{
  struct mj_trajectory_traj_data
  {
    Eigen::MatrixXd pos;
    Eigen::MatrixXd vel;
    Eigen::MatrixXd ace;
  };

  struct mj_trajectory_task_config
  {
    /* Position gain for trajectory task */
    double gain_pos = 1000;
    /* Velocity gain for trajectory task */
    double gain_vel = 100;
    /* Weight for trajectory task */
    double weight = 1e3;
  };

  struct mj_trajectory_config
  {
    mj_trajectory_config() {}
    /* Constructor
     *
     * Loads the associated datafile:
     * DATA_PATH + name.txt
     */
    mj_trajectory_config(const std::string & name);
    /* How many time the trajectory should be repeated */
    double count = 15.0;
    /* Task configuration */
    mj_trajectory_task_config conf;
    /* Data for trajectory playing */
    mj_trajectory_traj_data data;
  };

  class mj_trajectory
  {
   public:
    mj_trajectory(mc_solver::QPSolver & solver, mj_trajectory_config & config);

    ~mj_trajectory();

    bool update();
  public:
    mc_solver::QPSolver & solver;
    mj_trajectory_config & config;

    std::shared_ptr<tasks::qp::PositionTask> positionTask;
    std::shared_ptr<tasks::qp::TrajectoryTask> trajTask;
    Eigen::Vector3d initPos;
    Eigen::Vector3d refVel;
    long currentIndex = 0;
    double traj_count = 0.0;
    /* Build a static map of configuration, this allows us to load all the
     * necessary configurations at the controller load*/
    static std::map<std::string, mj_trajectory_config> traj_configs;
  };
}

#endif