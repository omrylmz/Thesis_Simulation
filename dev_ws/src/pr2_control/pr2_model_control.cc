#include <cmath>
#include <fstream>
#include <chrono>
#include <thread>
#include <functional>
#include <type_traits>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <nlohmann/json.hpp>
#include <range/v3/all.hpp>

using json = nlohmann::json;
// using namespace std;

enum class State
{
  PATH_SELECT,
  WAYPOINT_SELECT,
  ANGLE_CONTROL,
  LINEAR_CONTROL,
  GRASPING_SELECT,
  GRASPING
};

std::ostream &operator<<(std::ostream &os, const State &obj)
{
  switch (static_cast<std::underlying_type<State>::type>(obj))
  {
  case 0:
    os << "PATH_SELECT";
    break;
  case 1:
    os << "WAYPOINT_SELECT";
    break;
  case 2:
    os << "ANGLE_CONTROL";
    break;
  case 3:
    os << "LINEAR_CONTROL";
    break;
  case 4:
    os << "GRASPING_SELECT";
    break;
  case 5:
    os << "GRASPING";
    break;
  default:
    os << "INVALID STATE";
    break;
  }
  return os;
}

namespace gazebo
{

  class ModelPush : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->robot = _parent;
      this->world = this->robot->GetWorld();
      this->baseLink = this->robot->GetChildLink("base_footprint");
      this->state = State::PATH_SELECT;
      this->pidTheta = common::PID(5, 0, 0.5, 5, -5, 5, -5);
      this->isPlanLoaded = false;
      this->isAttached = false;
      this->inRelease = false;

      // Current step, attach start, grasp end, detach start, delivery end
      this->graspSteps = {
          {"CurrentStep", 0},
          {"AttachStart", 4},
          {"GraspingEnd", 6},
          {"DetachStart", 8},
          {"ReleaseEnd", 12},
      };

      this->graspCfgs = {
          // Steps
          {
              {"l_shoulder_pan_joint", 0.3},  // -0.71 => 2.28
              {"r_shoulder_pan_joint", -0.3}, // -2.28 => 0.71
          },
          {
              {"l_shoulder_lift_joint", 1}, // -0.52 => 1.39
              {"r_shoulder_lift_joint", 1}, // -0.52 => 1.39
          },
          {
              {"l_elbow_flex_joint", -0.35}, // -2.32 => 0.0
              {"r_elbow_flex_joint", -0.35}  // -2.32 => 0.0
          },
          {
              {"l_shoulder_pan_joint", 0.0},  // -0.71 => 2.28
              {"r_shoulder_pan_joint", -0.0}, // -2.28 => 0.71
          },
          // Attaching
          {
              {"l_shoulder_lift_joint", -0.5}, // -0.52 => 1.39
              {"r_shoulder_lift_joint", -0.5}, // -0.52 => 1.39
          },
          {
              {"l_elbow_flex_joint", -1}, // -2.32 => 0.0
              {"r_elbow_flex_joint", -1}  // -2.32 => 0.0
          },
          // Graspind ends
          {
              {"l_elbow_flex_joint", -0.0}, // -2.32 => 0.0
              {"r_elbow_flex_joint", -0.0}  // -2.32 => 0.0
          },
          {
              {"l_shoulder_lift_joint", 0.0}, // -0.52 => 1.39
              {"r_shoulder_lift_joint", 0.0}, // -0.52 => 1.39
          },
          // Detaching
          {
              {"l_shoulder_pan_joint", 0.1},  // -0.52 => 1.39
              {"r_shoulder_pan_joint", -0.1}, // -0.52 => 1.39
          },
          {
              {"l_elbow_flex_joint", -2.2}, // -2.32 => 0.0
              {"r_elbow_flex_joint", -2.2}  // -2.32 => 0.0
          },
          {
              {"l_shoulder_lift_joint", 1.25}, // -0.52 => 1.39
              {"r_shoulder_lift_joint", 1.25}, // -0.52 => 1.39
          },
          {
              {"l_shoulder_pan_joint", 0.0},  // -0.52 => 1.39
              {"r_shoulder_pan_joint", -0.0}, // -0.52 => 1.39
          },
          // Releasing ends
      };

      this->resetPoses();

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));
    }

    // Called by the world update start event
  public:
    void OnUpdate()
    {
      if (this->world->Iterations() < 1500)
      {
        return;
      }
      if (!this->isPlanLoaded)
      {
        std::ifstream planFile;
        planFile.open("/home/omer/Workspace/dev_ws/src/pr2_control/motion_plan.json");
        if (planFile.fail())
        {
          // std::this_thread::sleep_for(std::chrono::milliseconds(100));
          std::cout << "Waiting for the plan...\n";
          return;
        }
        json data = json::parse(planFile);
        this->paths = data.get<std::deque<std::deque<std::tuple<double, double>>>>();
        this->isPlanLoaded = true;
      }

      while (true)
      {
        switch (this->state)
        {
        case State::PATH_SELECT:

          if (this->paths.size() == 0)
          {
            std::cout << "ALL MOTIONS ARE COMPLETED";
            return;
          }
          else
          {
            this->waypoints = this->paths.front();
            this->paths.pop_front();
            this->state = State::WAYPOINT_SELECT;
          }
          break;

        case State::WAYPOINT_SELECT:
          std::cout << "WAYPOINTS LEFT: ";
          ranges::for_each(this->waypoints, [](const auto &w)
                           { std::cout << " (" << std::get<0>(w) << ", " << std::get<1>(w) << ") "; });
          std::cout << std::endl;
          this->pidTheta.Reset();
          if (this->waypoints.size() == 1)
          {
            this->pidTheta.Reset();
            this->target = this->waypoints.front();
            this->state = State::GRASPING_SELECT;
          }
          else
          {
            this->target = this->waypoints.front();
            this->waypoints.pop_front();
            this->reroute(this->target);
            this->state = State::ANGLE_CONTROL;
          }
          break;

        case State::ANGLE_CONTROL:
          this->pidTheta.Reset();
          this->pidTheta.SetPGain(10);
          this->pidTheta.SetIGain(0.5);
          this->pidTheta.SetDGain(0);
          this->reroute(this->target);
          if (std::abs(this->angleDiff(this->theta, this->yaw)) < 0.02)
          {
            this->state = State::LINEAR_CONTROL;
          }
          else
          {
            this->handleRelativePoses();
            auto ang_input = this->pidTheta.Update(this->yaw - this->theta, 0.04);
            auto linear = ignition::math::Vector3d(0.0, 0.0, 0.0);
            auto angular = ignition::math::Vector3d(0.0, 0.0, ang_input);
            this->robot->SetWorldTwist(linear, angular, true);
            this->handleAttached();
            if (this->graspSteps["CurrentStep"] > 1)
            {
              this->fixBase();
            }
            return;
          }
          break;

        case State::LINEAR_CONTROL:
          this->pidTheta.Reset();
          this->pidTheta.SetPGain(10);
          this->pidTheta.SetIGain(2);
          this->pidTheta.SetDGain(0.0);
          this->reroute(this->target);
          if (std::abs(this->angleDiff(this->theta, this->yaw)) > 0.2)
          {
            this->state = State::ANGLE_CONTROL;
          }
          else if (std::abs(this->linear) < 0.003 || (std::abs(this->linear) < 0.05 && this->waypoints.size() > 1))
          {
            this->state = State::WAYPOINT_SELECT;
          }
          else
          {
            this->handleRelativePoses();
            auto ang_input = this->pidTheta.Update(this->yaw - this->theta, 0.04);
            auto linear = ignition::math::Vector3d(std::get<0>(this->direction) * 1, std::get<1>(this->direction) * 1, 0.0);
            auto angular = ignition::math::Vector3d(0.0, 0.0, ang_input);

            this->robot->SetWorldTwist(linear, angular, true);
            this->fixBase();
            this->handleAttached();
            return;
          }
          break;

        case State::GRASPING_SELECT:
          if (this->waypoints.size() == 1)
          {
            this->pidTheta.SetPGain(10);
            this->pidTheta.SetIGain(2);
            this->pidTheta.SetDGain(0.0);
            this->reroute(this->target);
            if (std::abs(this->angleDiff(this->theta, this->yaw)) > 0.003)
            {
              this->handleRelativePoses();
              auto ang_input = this->pidTheta.Update(this->yaw - this->theta, 0.04);
              auto linear = ignition::math::Vector3d(0.0, 0.0, 0.0);
              auto angular = ignition::math::Vector3d(0.0, 0.0, ang_input);
              this->robot->SetWorldTwist(linear, angular, true);
              this->fixBase();
              this->handleAttached();
              return;
            }
            this->waypoints.pop_front();
          }
          std::cout << "GRASP STEP: " << this->graspSteps["CurrentStep"] << std::endl;
          if (this->graspSteps["AttachStart"] == this->graspSteps["CurrentStep"] && !this->isAttached)
          {
            this->isAttached = true;
            this->graspInitPose = this->findGraspCenter();
            this->graspOffSet = this->findOffSet();
          }
          else if (this->graspSteps["GraspingEnd"] == this->graspSteps["CurrentStep"] && !this->inRelease)
          {
            std::cout << "GRASPING ENDED!!!\n";
            this->inRelease = true;
            if (this->attachedRelPoses.size() == 0)
            {
              for (auto lnk : this->robot->GetLinks())
              {
                this->attachedRelPoses[lnk->GetName()] = lnk->RelativePose();
              }
            }
            this->state = State::PATH_SELECT;
          }
          else if (this->graspSteps["DetachStart"] == this->graspSteps["CurrentStep"] && this->isAttached)
          {
            this->isAttached = false;
          }
          else if (this->graspSteps["ReleaseEnd"] == this->graspSteps["CurrentStep"])
          {
            this->graspSteps["CurrentStep"] = 0;
            std::cout << "RELEASE ENDED!!!\n";
            this->inRelease = false;
            this->state = State::PATH_SELECT;
          }
          else
          {
            this->findCurrentGraspingGoals();
            this->pidArms.clear();
            for (auto i : ranges::views::iota(0, int(this->currGraspCfg.size())))
            {
              this->pidArms.push_back(common::PID(10, 0.5, 0, 2, -2, 0.002, -0.002));
            }
            this->state = State::GRASPING;
          }
          break;

        case State::GRASPING:
          std::deque<std::tuple<gazebo::physics::JointPtr, double, double>> jointController;
          for (auto graspCfg : this->currGraspCfg)
          {
            jointController.push_back(std::make_tuple(this->robot->GetJoint(std::get<0>(graspCfg)), std::get<1>(graspCfg),
                                                      this->robot->GetJoint(std::get<0>(graspCfg))->Position() - std::get<1>(graspCfg)));
          }
          if (ranges::all_of(jointController, [](auto &jntCntrl)
                             { if (std::abs(std::get<2>(jntCntrl)) < 0.02) {return true;} else {return false;} }))
          {
            // for (const auto& [i, jntCntrl] : jointController | ranges::views::enumerate)
            // {
            //   std::get<0>(jntCntrl)->SetPosition(0, std::get<1>(jntCntrl));
            // }
            for (const auto &jntCntrl : jointController)
            {
              std::get<0>(jntCntrl)->SetPosition(0, std::get<1>(jntCntrl));

              this->fixBase();
              this->handleAttached();
            }
            this->state = State::GRASPING_SELECT;
            return;
          }
          else
          {
            auto cmds = std::vector<double>();
            for (int i = 0; i < jointController.size(); ++i)
            {
              auto cmd = this->pidArms[i].Update(std::get<2>(jointController[i]), 0.04);
              // std::get<0>(jointController[i])->SetVelocity(0, cmd);
              std::get<0>(jointController[i])->SetPosition(0, std::get<0>(jointController[i])->Position() + cmd);

              this->fixBase();
              this->handleAttached();
            }
            return;
          }
          break;
        }
      }

    }

    // Handle relative poses
  private:
    void handleRelativePoses()
    {
      if (this->inRelease)
      {
        for (auto lnk : this->robot->GetLinks())
        {
          lnk->SetRelativePose(this->attachedRelPoses[lnk->GetName()]);
        }
      }
      else
      {
        for (auto lnk : this->robot->GetLinks())
        {
          lnk->SetRelativePose(this->releasedRelPoses[lnk->GetName()]);
        }
      }
    }

    // Handle attached object
  private:
    void handleAttached()
    {
      if (this->isAttached)
      {
        auto graspCenter = this->findGraspCenter();
        auto x = graspCenter.X();
        auto y = graspCenter.Y();
        auto z = graspCenter.Z();
        auto yaw = graspCenter.Yaw();
        auto poseTarget = ignition::math::Pose3d(x, y, z, 0, 0, yaw);
        this->world->ModelByName(this->attachedBoxName)->SetWorldPose(graspCenter);
      }
    }

    // Center of hands
  private:
    ignition::math::Pose3d findGraspCenter()
    {
      auto ll_pose = this->robot->GetLink("l_gripper_l_finger_tip_link")->WorldPose();
      auto lr_pose = this->robot->GetLink("l_gripper_r_finger_tip_link")->WorldPose();
      auto rl_pose = this->robot->GetLink("r_gripper_l_finger_tip_link")->WorldPose();
      auto rr_pose = this->robot->GetLink("r_gripper_r_finger_tip_link")->WorldPose();

      auto x = (ll_pose.X() + lr_pose.X() + rl_pose.X() + rr_pose.X()) / 4;
      auto y = (ll_pose.Y() + lr_pose.Y() + rl_pose.Y() + rr_pose.Y()) / 4;
      auto z = (ll_pose.Z() + lr_pose.Z() + rl_pose.Z() + rr_pose.Z()) / 4;
      auto roll = (ll_pose.Roll() + lr_pose.Roll() + rl_pose.Roll() + rr_pose.Roll()) / 4;
      auto pitch = (ll_pose.Pitch() + lr_pose.Pitch() + rl_pose.Pitch() + rr_pose.Pitch()) / 4;
      auto yaw = (ll_pose.Yaw() + lr_pose.Yaw() + rl_pose.Yaw() + rr_pose.Yaw()) / 4;
      return ignition::math::Pose3d(x, y, z, roll, pitch, yaw);
    }

    // Offset between box and hands' center
  private:
    ignition::math::v6::Pose3d findOffSet()
    {

      double minDistance = 100000000;
      for (const auto &model : this->world->Models())
      {
        if (model->GetName().rfind("unit_box", 0) == 0)
        {
          auto xDiff = model->WorldPose().X() - this->graspInitPose.X();
          auto yDiff = model->WorldPose().Y() - this->graspInitPose.Y();
          double distance = xDiff * xDiff + yDiff * yDiff;
          if (distance < minDistance && distance < 0.2)
          {
            this->attachedBoxName = model->GetName();
          }
        }
      }
      std::cout << "ATTACHED: " << this->attachedBoxName << std::endl
                << std::endl
                << std::endl;
      auto boxPose = this->world->ModelByName(this->attachedBoxName)->WorldPose();
      auto x = boxPose.X() - this->graspInitPose.X();
      auto y = boxPose.Y() - this->graspInitPose.Y();
      auto z = boxPose.Z() - this->graspInitPose.Z();
      auto roll = boxPose.Roll() - this->graspInitPose.Roll();
      auto pitch = boxPose.Pitch() - this->graspInitPose.Pitch();
      auto yaw = boxPose.Yaw() - this->graspInitPose.Yaw();
      return ignition::math::v6::Pose3d(x, y, z, roll, pitch, yaw);
    }

    // Grasp
  private:
    void findCurrentGraspingGoals()
    {
      this->currGraspCfg = this->graspCfgs[this->graspSteps["CurrentStep"]];
      this->graspSteps["CurrentStep"] = this->graspSteps["CurrentStep"] + 1;
    }

    // Should be called when target or pose updated
  private:
    void reroute(std::tuple<double, double> target)
    {
      auto currPose = this->robot->WorldPose();
      this->x = currPose.X();
      this->y = currPose.Y();
      this->yaw = currPose.Yaw();
      auto diffX = std::get<0>(target) - this->x;
      auto diffY = std::get<1>(target) - this->y;
      this->linear = std::sqrt(diffX * diffX + diffY * diffY);
      this->theta = std::atan2(diffY, diffX);
      if (std::abs(this->linear) > 10)
      {
        this->linear = this->linear / std::abs(this->linear) * 1;
      }
      auto xVel = this->linear * std::cos(this->theta);
      auto yVel = this->linear * std::sin(this->theta);
      this->direction = std::make_tuple(xVel, yVel);
    }

    // Fix base
  private:
    void fixBase()
    {
      auto basePose = this->baseLink->WorldPose();
      double x = basePose.X();
      double y = basePose.Y();
      double z = basePose.Z();
      double yaw = basePose.Yaw();
      auto fixedPose = ignition::math::v6::Pose3d(x, y, z, 0, 0, yaw);
    }

    // Reset poses
  private:
    void resetPoses()
    {
      ranges::for_each(this->robot->GetJoints(), [](const auto &jnt)
                       { std::cout << jnt->GetName() << ": " << jnt->Position() << std::endl; });
      this->robot->GetJoint("l_forearm_roll_joint")->SetPosition(0, 1.57);
      this->robot->GetJoint("r_forearm_roll_joint")->SetPosition(0, -1.57);

      this->robot->GetJoint("l_elbow_flex_joint")->SetPosition(0, -1.0);
      this->robot->GetJoint("r_elbow_flex_joint")->SetPosition(0, -1.0);
      this->robot->GetJoint("l_shoulder_lift_joint")->SetPosition(0, -0.5);
      this->robot->GetJoint("r_shoulder_lift_joint")->SetPosition(0, -0.5);

      // Record attached relative poses of each entity for resetting
      for (auto lnk : this->robot->GetLinks())
      {
        this->attachedRelPoses[lnk->GetName()] = lnk->RelativePose();
      }
      this->robot->GetJoint("l_elbow_flex_joint")->SetPosition(0, -2.2);
      this->robot->GetJoint("r_elbow_flex_joint")->SetPosition(0, -2.2);
      this->robot->GetJoint("l_shoulder_lift_joint")->SetPosition(0, 1.25);
      this->robot->GetJoint("r_shoulder_lift_joint")->SetPosition(0, 1.25);

      // Record released relative poses of each entity for resetting
      for (auto lnk : this->robot->GetLinks())
      {
        std::cout << lnk->GetName() << " ====>>> " << lnk->RelativePose() << " |||| " << lnk->WorldPose() << std::endl;
        this->releasedRelPoses[lnk->GetName()] = lnk->RelativePose();
      }
    }

    // Restrict angle difference between pi and -pi
  private:
    double angleDiff(double alpha, double beta)
    {
      if (3.1416 > alpha - beta && alpha - beta > -3.1416)
      {
        return alpha - beta;
      }
      else if (alpha - beta > 3.1416)
      {
        return angleDiff(alpha - 3.1416, beta);
      }
      else
      {
        return angleDiff(alpha + 3.1416, beta);
      }
    }

    // Grasp start pose
  private:
    ignition::math::v6::Pose3d graspInitPose;

    // Grasp offset
  private:
    ignition::math::v6::Pose3d graspOffSet;

    // In release phase or not
  private:
    bool inRelease;

    // Should attach or not
  private:
    bool isAttached;

    // Name of attached box
  private:
    std::string attachedBoxName;

    // Current attach step
  private:
    std::map<std::string, int> graspSteps;

    // Linear velocity input to apply
  private:
    double linear;

    // Pose x
  private:
    double x;

    // Pose y
  private:
    double y;

    // Pose yaw
  private:
    double yaw;

    // Velocity to apply
  private:
    std::tuple<double, double, double> velocity;

    // Error angle theta
  private:
    double theta;

    // Direction vector
  private:
    std::tuple<double, double> direction;

    // Target point
  private:
    std::tuple<double, double> target;

    // Waypoints
  private:
    std::deque<std::tuple<double, double>> waypoints;

    // Current grasping configuration
  private:
    std::deque<std::string> currFixed;

    // Current grasping configuration
  private:
    std::deque<std::tuple<std::string, double>> currGraspCfg;

    // Grasping configurations
  private:
    std::deque<std::deque<std::tuple<std::string, double>>> graspCfgs;

    // Paths
  private:
    std::deque<std::deque<std::tuple<double, double>>> paths;

    // State
  private:
    State state;

    // Check if plan file is ready
  private:
    bool isPlanLoaded;

    // PID controller for arms
  private:
    std::vector<common::PID> pidArms;

    // PID controller for body
  private:
    common::PID pidTheta;

  private:
    // Initial relative positions
    std::map<std::string, ignition::math::Pose3d> releasedRelPoses;

  private:
    // Initial relative positions
    std::map<std::string, ignition::math::Pose3d> attachedRelPoses;

    // Base link
  private:
    gazebo::physics::LinkPtr baseLink;

    // Pointer to the model
  private:
    physics::ModelPtr robot;

    // Pointer to the world
  private:
    physics::WorldPtr world;

    // Pointer to the update event connection
  private:
    event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
