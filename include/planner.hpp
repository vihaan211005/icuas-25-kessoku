#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <octomap/octomap.h>
#include <ompl/config.h>
#include <Eigen/Geometry>

// #include "fcl/fcl.h"
#include "traversal.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

octomap::point3d center_;
octomap::OcTree* octree_;
// std::vector<Eigen::Vector3d> pos_;

class Planner {
public:
    Planner(octomap::OcTree* tree, Bounds b, rclcpp::Logger logger)
        : bounds_(b), logger_(logger) {
        octree_ = tree;

        // fcl::OcTree* tree_ = new fcl::OcTree(std::shared_ptr<const octomap::OcTree>(tree));
        // tree_obj = std::shared_ptr<fcl::CollisionGeometry<fcl::OcTree>>(tree);

        // for(int i = 0; i < 4; i++){
        //     quad_obj[i] = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.10, 0.10, 0.03));
        // }
        // quadcopter = std::shared_ptr<fcl::CollisionGeometry>(new fcl::Box(0.10, 0.10, 0.03));

        space = std::make_shared<ob::SE3StateSpace>();

        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, bounds_.min[0]);
        bounds.setHigh(0, bounds_.max[0]);        
        bounds.setLow(1, bounds_.min[1]);
        bounds.setHigh(1, bounds_.max[1]);        
        bounds.setLow(2, bounds_.min[2]);
        bounds.setHigh(2, bounds_.max[2]);        

        // Set bounds directly on the SE3StateSpace
        space->setBounds(bounds);

        si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));

        pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));
    }

    int runPlanner(const Eigen::Vector4d& start, const Eigen::Vector4d& goal, std::vector<Eigen::Vector4d>& pathArray) {
        ob::ScopedState<ob::SE3StateSpace> startState(space);
        ob::ScopedState<ob::SE3StateSpace> goalState(space);

        // Set the start state
        startState->setX(start[0]);
        startState->setY(start[1]);
        startState->setZ(start[2]);
        Eigen::Quaterniond startQuat = yawToQuaternion(start[3]);
        startState->rotation().x = startQuat.x();
        startState->rotation().y = startQuat.y();
        startState->rotation().z = startQuat.z();
        startState->rotation().w = startQuat.w();

        // Set the goal state
        Eigen::Quaterniond goalQuat = yawToQuaternion(goal[3]);
        goalState->rotation().x = goalQuat.x();
        goalState->rotation().y = goalQuat.y();
        goalState->rotation().z = goalQuat.z();
        goalState->rotation().w = goalQuat.w();

        std::cout << "start: " << "(" << start[0] << "," << start[1] << "," << start[2] << "," << start[3] << ") " 
                  << "goal: " << "(" << goal[0] << "," << goal[1] << "," << goal[2] << "," << goal[3] << ")" << std::endl;
        this->printOctreeBounds();

        if(!isStateValid(startState.get())){
            std::cout << "Start state is invalid" << std::endl;
            throw std::runtime_error("start state is invalid");
        }

        pdef->clearSolutionPaths();  
        pdef->clearStartStates();
        pdef->addStartState(startState);
        pdef->clearGoal();
        pdef->setGoalState(goalState);
        
        ob::PlannerPtr plan(new og::InformedRRTstar(si));
        plan->setProblemDefinition(pdef);
        plan->setup();
        
        ob::PlannerStatus solved = plan->solve(2);
        std::cout << "Solved!" << std::endl;

        if (solved) {
            std::cout << "Found a solution!" << std::endl;
            og::PathGeometric path(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            path.printAsMatrix(std::cout);

            for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                const auto *state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
                Eigen::Quaterniond stateQuat;
                stateQuat.x() = goalState->rotation().x;
                stateQuat.y() = goalState->rotation().y;
                stateQuat.z() = goalState->rotation().z;
                stateQuat.w() = goalState->rotation().w;
                Eigen::Vector4d p(state->getX(), state->getY(), state->getZ(), quaternionToYaw(stateQuat));
                pathArray.push_back(p);
            }
        } else {
            std::cerr << "No solution found." << std::endl;
            return 1;
        }
        return 0;
    }

    void setCenter(const octomap::point3d center) {
        center_ = center;
    }
    
    // void setPosition(std::vector<Eigen::Vector3d> pos){
    //     pos_ = pos;
    // }

    void printOctreeBounds() {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        std::cout << "Octree Bounds:\n";
        std::cout << "  Min: (" << min_x << ", " << min_y << ", " << min_z << ")\n";
        std::cout << "  Max: (" << max_x << ", " << max_y << ", " << max_z << ")\n";
    }

private:
    bool isStateValid(const ob::State *state) const {
        const auto *se3State = state->as<ob::SE3StateSpace::StateType>();
        double x = se3State->getX();
        double y = se3State->getY();
        double z = se3State->getZ();

        octomap::point3d center(center_.x(), center_.y(), center_.z());

        octomap::OcTreeNode *node = octree_->search(x, y, z);

        if (node != nullptr && octree_->isNodeOccupied(node)) {
            return false;
        }
        
        octomap::point3d target(x, y, z);
        octomap::point3d hit;

        bool collision = octree_->castRay(center, target - center, hit, true, (target - center).norm() - 0.01);
        if(collision){
            return false;
        }

        
		// fcl::CollisionObject quadcopterObject(quadcopter);

        // const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
		// const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);
		// fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
		// fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
		// quadcopterObject.setTransform(rotation, translation);

        // for(int i = 0; i < 4; i++){
        //     fcl::Vec3f translation(pos->values[0],pos->values[1],pos->values[2]);
        //     fcl::Quaternion3f rotation(rot->w, rot->x, rot->y, rot->z);
        //     quadcopterObject.setTransform(rotation, translation);
        // }

		// fcl::CollisionRequest requestType(1,false,1,false);

		// fcl::CollisionResult collisionResult;
		// fcl::collide(&quad5_obj, &treeObj, requestType, collisionResult);

        return true;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr){
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }

    Eigen::Quaterniond yawToQuaternion(double yaw) const {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    double quaternionToYaw(const Eigen::Quaterniond& q) const {
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler[2]; // Yaw is the third component
    }

    Bounds bounds_;
    rclcpp::Logger logger_;
    ob::SpaceInformationPtr si;
    std::shared_ptr<ob::SE3StateSpace> space;
    ob::ProblemDefinitionPtr pdef;

    // std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj;
    // std::vector<std::shared_ptr <fcl::CollisionGeometry<double>> > quad_obj(4);
    // std::shared_ptr <fcl::CollisionGeometry<double>> quadcopter;
};