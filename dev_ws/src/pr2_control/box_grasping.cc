#include <ignition/math/Pose3.hh>
#include <gazebo/gazebo.hh>
#include "gazebo/physics/physics.hh"
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "gazebo/common/common.hh"

using namespace std;

namespace gazebo
{
    class BoxGrasping : public WorldPlugin
    {
    public:
        BoxGrasping() : WorldPlugin()
        {
            printf("Hello World!\n");
        }

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            // Get the pointer to the world
            this->world = _world;

            // Get the pointer to the unit_box
            this->unit_box = this->world->ModelByName("unit_box");

            // Get the pointer to the robot and fingers
            this->robot = this->world->ModelByName("pr2");
            this->l_gripper_l_finger_tip_link = this->robot->GetLink("l_gripper_l_finger_tip_link");
            this->l_gripper_r_finger_tip_link = this->robot->GetLink("l_gripper_r_finger_tip_link");
            this->r_gripper_l_finger_tip_link = this->robot->GetLink("r_gripper_l_finger_tip_link");
            this->r_gripper_r_finger_tip_link = this->robot->GetLink("r_gripper_r_finger_tip_link");

            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&BoxGrasping::OnUpdate, this));
        }

    public:
        void OnUpdate()
        {
            cout << "gggggggggggggggggggggg\n";
            auto ll_pose = this->l_gripper_l_finger_tip_link->WorldPose();
            auto lr_pose = this->l_gripper_r_finger_tip_link->WorldPose();
            auto rl_pose = this->r_gripper_l_finger_tip_link->WorldPose();
            auto rr_pose = this->r_gripper_r_finger_tip_link->WorldPose();
            
            // this->unit_box->SetWorldPose((ll_pose + rl_pose) / 2.0);
        }

        // Pointer to the unit_box
    private:
        physics::ModelPtr unit_box;

        // Pointers to the fingers
    private:
        physics::LinkPtr l_gripper_l_finger_tip_link;
        physics::LinkPtr l_gripper_r_finger_tip_link;
        physics::LinkPtr r_gripper_l_finger_tip_link;
        physics::LinkPtr r_gripper_r_finger_tip_link;

        // Pointer to the robot
    private:
        physics::ModelPtr robot;

        // Pointer to the world
    private:
        physics::WorldPtr world;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    GZ_REGISTER_WORLD_PLUGIN(BoxGrasping)
}