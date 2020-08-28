///
/// ... mc_hri1_controller ...
///

#include "mc_hri1_controller.h"

namespace mc_control
{
    using namespace rbd;

    MCHRI1Controller::MCHRI1Controller(const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt)
    : MCController(robot, dt)
    {
        std::array<double, 3> damper = {{0.01, 0.001, 0.01}};
        // KinematicsConstraint(robots, robotIndex, timeStep, damper, velocityPercent)
        kinematicsConstraint = mc_solver::KinematicsConstraint(robots(), 0, timeStep, damper, 0.8);

        // add collision ConstraintSet
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK3", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK4", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK6", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK3", "LLEG_LINK3", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK4", "LLEG_LINK3", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK6", "LLEG_LINK3", 0.1, 0.05, 0});

        qpsolver->addConstraintSet(contactConstraint);
        qpsolver->addConstraintSet(kinematicsConstraint);
        qpsolver->addConstraintSet(selfCollisionConstraint);

        // set RArm wrist jointGain
        postureTask->jointsGains(robots().mbs(), {{"RARM_JOINT5", 1e3}});
        qpsolver->addTask(postureTask.get());

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        efTask.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(),5.0,1e3));
        oriTask.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
        solver().addTask(comTask);

        elbow_pos.reset(new mc_tasks::PositionTask("RARM_LINK3", robots(), 0, 1.0, 1e4));
        Eigen::VectorXd dimW(3);
        dimW << 0., 0., 2.;//0., 0., 1.;
        elbow_pos->dimWeight(dimW);

        LOG_SUCCESS("MCHRI1Controller init done")
    }

    void MCHRI1Controller::reset(const ControllerResetData & reset_data)
    {
        auto q = reset_data.q;
        std::cout << "q[0] ";
        for(const auto & qi : q[0])
        {
          std::cout << qi << ", ";
        }
        std::cout << std::endl;
        q[0] = {0.991445, -0, -0, 0.130526, -0.275, -0.05, 0.825336};
        MCController::reset({q});

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        efTask->reset();
        comTask->reset();
        elbow_pos->reset();
    }

    bool MCHRI1Controller::run()
    {
      bool ret = MCController::run();
      if(trajectoryTask)
      {
        if(trajectoryTask->update())
        {
          ready = true;
        }
      }
      return ret;
    }

    void MCHRI1Controller::init_pos()
    {
      /* Set the hand in its initial configuration */
      std::vector<double> rArm_Joints_q =
      {-0.4, -0.55, 0.55, -0.65, -1.2, 0.37, -0.1, -0.00};

      for(size_t i = 0; i < 8; ++i)
      {
        std::stringstream ss;
        ss << "RARM_JOINT" << i;
        set_joint_pos(ss.str(), rArm_Joints_q[i]);
      }
      solver().addTask(efTask);

      Eigen::Vector3d initPos;
      initPos <<  0.392299, -0.508818, 1.1509;
      Eigen::Matrix3d initOri;
      initOri <<    -0.295298,  0.338036,  3.0605,
              -0.758028,  0.486418, -0.4345,
              -0.581542, -0.805685,  0.112602;

      efTask->set_ef_pose(sva::PTransformd(initOri, initPos));
    }

    void MCHRI1Controller::traj_task(const std::string & name)
    {
      ready = false;
      if(first_traj)
      {
        /* Remove the task added by init_pos */
        solver().removeTask(efTask);
        /* Set up task to lock the orientation and the elbow position */
        elbow_pos->reset();
        oriTask->reset();
        solver().addTask(elbow_pos);
        solver().addTask(oriTask);
      }
      trajectoryTask.reset(new mj_traj::mj_trajectory(solver(), mj_traj::mj_trajectory::traj_configs.at(name)));
    }

    std::ostream& MCHRI1Controller::log_header(std::ostream & os)
    {
      os << ";ef_pos_x";
      os << ";ef_pos_y";
      os << ";ef_pos_z";
      os << ";ef_vel_x";
      os << ";ef_vel_y";
      os << ";ef_vel_z";
      os << ";ef_ref_x";
      os << ";ef_ref_y";
      os << ";ef_ref_z";
      os << ";ef_velref_x";
      os << ";ef_velref_y";
      os << ";ef_velref_z";
      return os;
    }

    std::ostream& MCHRI1Controller::log_data(std::ostream & os)
    {
      Eigen::Vector3d pos = robot().mbc().bodyPosW[robot().bodyIndexByName("RARM_LINK7")].translation();
      Eigen::Vector3d vel = robot().mbc().bodyVelW[robot().bodyIndexByName("RARM_LINK7")].linear();
      Eigen::Vector3d ref(0,0,0);
      Eigen::VectorXd refVel = Eigen::VectorXd::Zero(3);
      if(trajectoryTask)
      {
        ref = trajectoryTask->positionTask->position();
        refVel = trajectoryTask->refVel;
      }
      os << ";" << pos.x();
      os << ";" << pos.y();
      os << ";" << pos.z();
      os << ";" << vel.x();
      os << ";" << vel.y();
      os << ";" << vel.z();
      os << ";" << ref.x();
      os << ";" << ref.y();
      os << ";" << ref.z();
      os << ";" << refVel.x();
      os << ";" << refVel.y();
      os << ";" << refVel.z();
      return os;
    }

    bool MCHRI1Controller::read_msg(std::string & msg)
    {
      std::stringstream ss; ss << msg;
      std::string token;
      ss >> token;

      /*sitting positon*/
      if(token == "sit")
      {
         MCController::set_joint_pos("RLEG_JOINT2",-1.7);
         MCController::set_joint_pos("LLEG_JOINT2",-1.7);

         MCController::set_joint_pos("RLEG_JOINT3", 1.6);
         MCController::set_joint_pos("LLEG_JOINT3", 1.6);

         MCController::set_joint_pos("RLEG_JOINT4", 0.0);
         MCController::set_joint_pos("LLEG_JOINT4", 0.0);

         return true;
      }

      // set initial pose
      if(token == "initial")
      {
        ///HEAD orientation in the beginning of exp
        MCController::set_joint_pos("HEAD_JOINT0", -0.3); //-ve to move head to robot's right
        MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down

        ///initial orientation and pos in the beginning of exp
        init_pos();
        return true;
      }
      /// open right gripper
      if(token == "open_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;
      }
      /// close right gripper
      if(token == "close_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }
      /// get RARM JOINTs
      std::vector<double>  get_RArm_Joints(8);
      if(token == "get_pose")
      {
        for (int i = 0; i<8; ++i)
        {
          std::stringstream ss;
          ss << "RARM_JOINT" << i;
          std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
        }
        std::cout<<std::endl;
        return true;
      }
      if(mj_traj::mj_trajectory::traj_configs.count(token))
      {
        if(ready)
        {
          traj_task(token);
        }
        else
        {
          LOG_ERROR("Current trajectory not finished")
          return false;
        }
        return true;
      }
      if(token == "stop")
      {
        if(trajectoryTask)
        {
          LOG_WARNING("Stopping after the next iteration")
          trajectoryTask->traj_count = trajectoryTask->config.count;
        }
      }
      LOG_WARNING("Cannot handle " << msg)
      return false;
    }
}
