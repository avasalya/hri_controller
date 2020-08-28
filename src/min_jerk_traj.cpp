///
/// ... min_jerk_traj ...
///

#include "min_jerk_traj.h"

/// min-jerk-trajectory namespace ///
namespace mj_traj
{
  mj_trajectory_config::mj_trajectory_config(const std::string & name)
  {
    std::string fn = std::string(DATA_PATH) + "/" + name + ".txt";
    std::ifstream ifs(fn);
    if(!ifs.is_open())
    {
      LOG_ERROR("Failed to open " << fn)
      throw("File not found");
    }
    double pt;
    std::vector<double> pts;
    while(ifs >> pt)
    {
      pts.push_back(pt);
    }
    data.pos = Eigen::MatrixXd(3, pts.size()/9);
    data.vel = Eigen::MatrixXd(3, pts.size()/9);
    data.ace = Eigen::MatrixXd(3, pts.size()/9);
    for(size_t i = 0; i < pts.size(); i += 9)
    {
      data.pos(0, i/9) = pts[i];
      data.pos(1, i/9) = pts[i+1];
      data.pos(2, i/9) = pts[i+2];
      data.vel(0, i/9) = pts[i+3];
      data.vel(1, i/9) = pts[i+4];
      data.vel(2, i/9) = pts[i+5];
      data.ace(0, i/9) = pts[i+6];
      data.ace(1, i/9) = pts[i+7];
      data.ace(2, i/9) = pts[i+8];
    }
  }

   std::map<std::string, mj_trajectory_config> mj_trajectory::traj_configs = []()
  {
    std::map<std::string, mj_trajectory_config> res;
    std::vector<std::string> prefix = {"s1", "s2", "s3", "s4", "mj", "mjt", "it1", "it2", "it3", "it4", "it5"};
    std::vector<std::string> suffix = {"pt45", "pt5", "pt55", "pt6", "pt65",
                                       "pt7", "pt75", "pt8", "pt85", "pt9", "pt95",
                                       "sec1", "sec2", "sec3","sec4",
                                       "sec1_pt5","sec2_pt5"};

    // pick runtime for trajectories  
    double nSecs = 40.0;
    std::map<std::string, double> count_per_suffix; 
    // run each trajectory for 'nSecs' seconds
    count_per_suffix["sec4"]    =  floor(nSecs/8);  //4.0sec
    count_per_suffix["sec3"]    =  floor(nSecs/6);  //3.0sec
    count_per_suffix["sec2_pt5"]=  floor(nSecs/5);  //2.5sec
    count_per_suffix["sec2"]    =  floor(nSecs/4);  //2.0sec
    count_per_suffix["sec1_pt5"]=  floor(nSecs/3);  //1.5sec
    count_per_suffix["sec1"]    =  floor(nSecs/2);  //1.0sec
    count_per_suffix["pt95"]    =  floor(nSecs/1.9);  //.95sec
    count_per_suffix["pt9"]     =  floor(nSecs/1.8);  //0.9sec
    count_per_suffix["pt85"]    =  floor(nSecs/1.7);  //.85sec
    count_per_suffix["pt8"]     =  floor(nSecs/1.6);  //0.8sec
    count_per_suffix["pt75"]    =  floor(nSecs/1.5);  //.75sec
    count_per_suffix["pt7"]     =  floor(nSecs/1.4);  //0.7sec
    count_per_suffix["pt65"]    =  floor(nSecs/1.3);  //.65sec
    count_per_suffix["pt6"]     =  floor(nSecs/1.2);  //0.6sec
    count_per_suffix["pt55"]    =  floor(nSecs/1.1);  //.55sec
    count_per_suffix["pt5"]     =  floor(nSecs/1.0);  //0.5sec
    count_per_suffix["pt45"]    =  floor(nSecs/0.9);  //.45sec

    for(const auto & p : prefix)
    {
      for(const auto & s : suffix)
      {
        std::string name = p + "_" + s;
        res[name] = mj_trajectory_config(name);
        res[name].count = count_per_suffix[s];
      }
    }
    return res;
  }();

  /* Do per-trajectory tuning here */

  mj_trajectory::mj_trajectory(mc_solver::QPSolver & s, mj_trajectory_config & conf)
  : solver(s), config(conf)
  {
    auto & robot = solver.robot();
    positionTask = std::make_shared<tasks::qp::PositionTask>(solver.robots().mbs(), 0, "RARM_LINK7", robot.mbc().bodyPosW[robot.bodyIndexByName("RARM_LINK7")].translation());
    Eigen::Vector3d dimW; dimW << 1, 1, 1;
    trajTask = std::make_shared<tasks::qp::TrajectoryTask>(solver.robots().mbs(), 0, positionTask.get(), config.conf.gain_pos, config.conf.gain_vel, dimW, config.conf.weight);
    /* Update position vector */
    initPos = positionTask->position();
    solver.addTask(trajTask.get());
  }

  mj_trajectory::~mj_trajectory()
  {
    solver.removeTask(trajTask.get());
  }

  bool mj_trajectory::update()
  {
    if(currentIndex < config.data.pos.cols())
    {
      positionTask->position(config.data.pos.col(currentIndex) + initPos - config.data.pos.col(0));
      trajTask->refVel(config.data.vel.col(currentIndex));
      refVel = config.data.vel.col(currentIndex);
      trajTask->refAccel(config.data.ace.col(currentIndex));
      currentIndex++;
      if(currentIndex == config.data.pos.cols())
      {
        traj_count++;
        LOG_INFO("Completed pass " << traj_count << " of " << config.count)
        if(traj_count == config.count)
        {
          return true;
        }
        else
        {
          currentIndex = 0;
        }
      }
      return false;
    }
    else
    {
      positionTask->position(initPos);
      trajTask->refVel(Eigen::Vector3d::Zero());
      refVel = Eigen::Vector3d::Zero();
      trajTask->refAccel(Eigen::Vector3d::Zero());
      return false;
    }
  }
}
