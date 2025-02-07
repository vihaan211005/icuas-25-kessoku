#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <octomap/octomap.h>
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/srv/get_octomap.hpp"

#include "traversal.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

class Planner {
public:
    Planner(octomap::OcTree* tree, Bounds b, rclcpp::Logger logger)
        : bounds_(b), logger_(logger), octree_(tree) {
        space = std::make_shared<ob::RealVectorStateSpace>(3);

        ob::RealVectorBounds bounds(3);
        bounds.setLow(0, bounds_.min[0]);
        bounds.setHigh(0, bounds_.max[0]);        
        bounds.setLow(1, bounds_.min[1]);
        bounds.setHigh(1, bounds_.max[1]);        
        bounds.setLow(2, bounds_.min[2]);
        bounds.setHigh(2, bounds_.max[2]);        
        space->setBounds(bounds);

        si = std::make_shared<ob::SpaceInformation>(space);
        si->setStateValidityChecker(std::bind(&Planner::isStateValid, this, std::placeholders::_1));

        pdef = std::make_shared<ob::ProblemDefinition>(si);
        pdef->setOptimizationObjective(Planner::getPathLengthObjWithCostToGo(si));
    }

    int runPlanner(const std::vector<double>& start, const std::vector<double>& goal, std::vector<Eigen::Vector3d>& pathArray) {


        ob::ScopedState<ob::RealVectorStateSpace> startState(space);
        ob::ScopedState<ob::RealVectorStateSpace> goalState(space);

        startState->values[0] = start[0];
        startState->values[1] = start[1];
        startState->values[2] = start[2];
        
        goalState->values[0] = goal[0];
        goalState->values[1] = goal[1];
        goalState->values[2] = goal[2];

        pdef->clearSolutionPaths();  
        pdef->clearStartStates();
        pdef->addStartState(startState);
        pdef->clearGoal();
        pdef->setGoalState(goalState);
        
        auto planner = std::make_shared<og::InformedRRTstar>(si);
        planner->setProblemDefinition(pdef);
        planner->setup();
        
        ob::PlannerStatus solved = planner->solve(ob::timedPlannerTerminationCondition(0.5));

        if (solved) {
            std::cout << "Found a solution!" << std::endl;
            og::PathGeometric path(dynamic_cast<const og::PathGeometric&>(*pdef->getSolutionPath()));
            path.printAsMatrix(std::cout);

            for (std::size_t i = 0; i < path.getStateCount(); ++i) {
                const auto *state = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                Eigen::Vector3d p(state->values[0], state->values[1], state->values[2]);
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

private:
    bool isStateValid(const ob::State *state) const {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        double z = realState->values[2];

        octomap::OcTreeNode *node = octree_->search(x, y, z);
        if (node != nullptr && octree_->isNodeOccupied(node)) {
            return false;
        }
        
        octomap::point3d target(x, y, z);
        octomap::point3d hit;
        bool collision = octree_->castRay(center_, target - center_, hit, true, (target - center_).norm() - 0.01);
        return !collision;
    }

    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr){
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }

    Bounds bounds_;
    octomap::point3d center_;
    rclcpp::Logger logger_;
    ob::SpaceInformationPtr si;
    std::shared_ptr<ob::RealVectorStateSpace> space;
    ob::ProblemDefinitionPtr pdef;
    octomap::OcTree* octree_;
};
