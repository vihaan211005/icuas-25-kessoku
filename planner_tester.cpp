#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <octomap/octomap.h>
#include <ompl/config.h>
#include <Eigen/Geometry>

#include "fcl/fcl.h"
// Planners
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
// #include "utils.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

static octomap::point3d center_;
static octomap::OcTree *octree_;
static std::vector<Eigen::Vector3d> pos_;

using namespace std;

class Bounds
{
public:
    Eigen::Vector3d min, max;
    Bounds(Eigen::Vector3d min = Eigen::Vector3d(0, 0, 0), Eigen::Vector3d max = Eigen::Vector3d(0, 0, 0)) : min(min), max(max) {}

    friend std::ostream &operator<<(std::ostream &os, const Bounds &b)
    {
        os << "Bounds(" << b.min << ", " << b.max << ")";
        return os;
    }
};

class Planner
{
public:
    Planner(octomap::OcTree *tree, Bounds b) : bounds_(b)
    {
        octree_ = tree;
        fcl::OcTree<double> *tree_ = new fcl::OcTree<double>(std::shared_ptr<const octomap::OcTree>(new octomap::OcTree(0.1)));
        tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree_);

        for (int i = 0; i < 4; i++)
        {
            quad_obj.push_back(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box(0.10, 0.10, 0.03)));
        }
        quadcopter = std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box(0.10, 0.10, 0.03));

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

    int runPlanner(const Eigen::Vector4d &start, const Eigen::Vector4d &goal)
    {
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
        goalState->setX(goal[0]);
        goalState->setY(goal[1]);
        goalState->setZ(goal[2]);
        Eigen::Quaterniond goalQuat = yawToQuaternion(goal[3]);
        goalState->rotation().x = goalQuat.x();
        goalState->rotation().y = goalQuat.y();
        goalState->rotation().z = goalQuat.z();
        goalState->rotation().w = goalQuat.w();

        this->printOctreeBounds();

        std::cout << startState->getX() << " " << startState->getY() << " " << startState->getZ() << std::endl;
        if (!isStateValid(startState.get()))
        {
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

        if (solved)
        {
            og::PathGeometric path(dynamic_cast<const og::PathGeometric &>(*pdef->getSolutionPath()));
            path.printAsMatrix(std::cout);

            for (std::size_t i = 0; i < path.getStateCount(); ++i)
            {
                const auto *state = path.getState(i)->as<ob::SE3StateSpace::StateType>();
                Eigen::Quaterniond stateQuat;
                stateQuat.x() = goalState->rotation().x;
                stateQuat.y() = goalState->rotation().y;
                stateQuat.z() = goalState->rotation().z;
                stateQuat.w() = goalState->rotation().w;
                Eigen::Vector4d p(state->getX(), state->getY(), state->getZ(), quaternionToYaw(stateQuat));
                // shared_queue_.push(p);
            }
        }
        else
        {
            return 1;
        }
        return 0;
    }

    void setCenter(const octomap::point3d center)
    {
        center_ = center;
    }

    void setPosition(std::vector<Eigen::Vector3d> pos)
    {
        pos_ = pos;
    }

    void printOctreeBounds()
    {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

    }

private:
    bool isStateValid(const ob::State *state) const
    {
        const auto *se3State = state->as<ob::SE3StateSpace::StateType>();
        double x = se3State->getX();
        double y = se3State->getY();
        double z = se3State->getZ();

        octomap::point3d center(center_.x(), center_.y(), center_.z());

        octomap::OcTreeNode *node = octree_->search(x, y, z);

        if (node != nullptr && octree_->isNodeOccupied(node))
        {
            return false;
        }

        octomap::point3d target(x, y, z);
        octomap::point3d hit;

        bool collision = octree_->castRay(center, target - center, hit, true, (target - center).norm() - 0.01);
        if (collision)
        {
            return false;
        }

        fcl::CollisionObject quadcopterObject(quadcopter);
        const ob::RealVectorStateSpace::StateType *pos = se3State->as<ob::RealVectorStateSpace::StateType>(0);
        const ob::SO3StateSpace::StateType *rot = se3State->as<ob::SO3StateSpace::StateType>(1);
        fcl::Vector3d translation(pos->values[0], pos->values[1], pos->values[2]);
        fcl::Quaterniond rotation(rot->w, rot->x, rot->y, rot->z);
        quadcopterObject.setTransform(rotation, translation);

        fcl::CollisionObject quad1Object(quad_obj[0]);
        fcl::Vector3d translation1(pos_[0].x(), pos_[0].y(), pos_[0].z());
        fcl::Quaterniond rotation1(1, 0, 0, 0);
        quad1Object.setTransform(rotation1, translation1);

        fcl::CollisionObject quad2Object(quad_obj[1]);
        fcl::Vector3d translation2(pos_[1].x(), pos_[1].y(), pos_[1].z());
        fcl::Quaterniond rotation2(1, 0, 0, 0);
        quad2Object.setTransform(rotation2, translation2);

        fcl::CollisionObject quad3Object(quad_obj[2]);
        fcl::Vector3d translation3(pos_[2].x(), pos_[2].y(), pos_[2].z());
        fcl::Quaterniond rotation3(1, 0, 0, 0);
        quad3Object.setTransform(rotation3, translation3);

        fcl::CollisionObject quad4Object(quad_obj[3]);
        fcl::Vector3d translation4(pos_[3].x(), pos_[3].y(), pos_[3].z());
        fcl::Quaterniond rotation4(1, 0, 0, 0);
        quad4Object.setTransform(rotation4, translation4);

        fcl::CollisionObject<double> treeObj((tree_obj));

        fcl::CollisionRequest<double> requestType(1, false, 1, false);

        fcl::CollisionResult<double> collisionResultTree;
        fcl::collide(&quadcopterObject, &treeObj, requestType, collisionResultTree);
        if (collisionResultTree.isCollision()){
            std::cout<<"Hello1\n";
            return false;}

        fcl::CollisionResult<double> collisionResultQuad1;
        fcl::collide(&quadcopterObject, &quad1Object, requestType, collisionResultQuad1);
        if (collisionResultQuad1.isCollision()){
            std::cout<<"Hello2\n";
            return false;}
        
        fcl::CollisionResult<double> collisionResultQuad2;
        fcl::collide(&quadcopterObject, &quad2Object, requestType, collisionResultQuad2);
        if (collisionResultQuad2.isCollision()){
            std::cout<<"Hello3\n";
            return false;}

        fcl::CollisionResult<double> collisionResultQuad3;
        fcl::collide(&quadcopterObject, &quad3Object, requestType, collisionResultQuad3);
        if (collisionResultQuad3.isCollision()){
            std::cout<<"Hello4\n";
            return false;}

        fcl::CollisionResult<double> collisionResultQuad4;
        fcl::collide(&quadcopterObject, &quad4Object, requestType, collisionResultQuad4);
        if (collisionResultQuad4.isCollision()){
            std::cout<<"Hello\n";
            return false;}

        return true;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr)
    {
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }

    Eigen::Quaterniond yawToQuaternion(double yaw) const
    {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    double quaternionToYaw(const Eigen::Quaterniond &q) const
    {
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
        return euler[2]; // Yaw is the third component
    }

    Bounds bounds_;
    ob::SpaceInformationPtr si;
    std::shared_ptr<ob::SE3StateSpace> space;
    ob::ProblemDefinitionPtr pdef;

    std::shared_ptr<fcl::CollisionGeometry<double>> tree_obj;
    std::vector<std::shared_ptr<fcl::CollisionGeometry<double>>> quad_obj;
    std::shared_ptr<fcl::CollisionGeometry<double>> quadcopter;

    // utils::sharedQueue<Eigen::Vector4d> &shared_queue_;
};

int main()
{
    octomap::OcTree *octree = new octomap::OcTree("city_1.binvox.bt");
    octomap::point3d hit;
    octomap::point3d start(25.576, 76.7281, 14.2089);
    octomap::point3d goal(26.388, 73.4804, 13.397);
    octomap::point3d center(32.0716, 56.4297, 3.85372);
    bool news = octree->castRay(center, start - center, hit, true, (center - start).norm() - 0.01);
    std::cout << news;
    octomap::OcTreeNode *node = octree->search(start);
    if (node != nullptr && octree->isNodeOccupied(node))
    {
        cout << "node was occupied!" << endl;
        std::cout << "false";
    }
    node = octree->search(goal);
    if (node != nullptr && octree->isNodeOccupied(node))
    {
        cout << "node was occupied!" << endl;
        std::cout << "false";
    }
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree->getMetricMin(min_x, min_y, min_z);
    octree->getMetricMax(max_x, max_y, max_z);

    Bounds bounds = Bounds(Eigen::Vector3d(10.1492, 9.33729, -0.405969), Eigen::Vector3d(97.0266, 114.889, 33.6954));

    Planner planner(octree, bounds);
    planner.setCenter(center);

    std::vector<Eigen::Vector3d> poses = {Eigen::Vector3d(0,0,0),Eigen::Vector3d(1,1,1),Eigen::Vector3d(2,2,2),Eigen::Vector3d(3,3,3)};
    planner.setPosition(poses);

    planner.runPlanner(Eigen::Vector4d(start.x(), start.y(), start.z(), -1), Eigen::Vector4d(goal.x(), goal.y(), goal.z(), 0));
    return 0;
}