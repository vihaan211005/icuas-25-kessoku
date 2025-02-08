#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <octomap/octomap.h>

#include "traversal.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;

octomap::point3d center_;
octomap::OcTree* octree_;

class Planner {
public:
    Planner(octomap::OcTree* tree, Bounds b, rclcpp::Logger logger)
        : bounds_(b), logger_(logger) {
        octree_ = tree;
        space = std::make_shared<ob::RealVectorStateSpace>(3);

        std::cout << bounds_;
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

    int runPlanner(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, std::vector<Eigen::Vector3d>& pathArray) {
        ob::ScopedState<ob::RealVectorStateSpace> startState(space);
        ob::ScopedState<ob::RealVectorStateSpace> goalState(space);

        startState->values[0] = start[0];
        startState->values[1] = start[1];
        startState->values[2] = start[2];
        
        goalState->values[0] = goal[0];
        goalState->values[1] = goal[1];
        goalState->values[2] = goal[2];

        std::cout << "start: " << "(" << start[0] << "," << start[1] << "," << start[2] << ") " << "goal: " << goal[0] << "," << goal[1] << "," << goal[2] << ")" << std::endl;
        printOctreeBounds();

        const auto *tmpState = startState.get()->as<ob::RealVectorStateSpace::StateType>();
        std::cout << "Start state: " << tmpState->values[0] << " " << tmpState->values[1] << " " << tmpState->values[2] << std::endl;

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

    void printOctreeBounds() const {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        std::cout << "Octree Bounds:\n";
        std::cout << "  Min: (" << min_x << ", " << min_y << ", " << min_z << ")\n";
        std::cout << "  Max: (" << max_x << ", " << max_y << ", " << max_z << ")\n";
    }
private:
    bool isStateValid(const ob::State *state) const {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        double z = realState->values[2];

        octomap::point3d center(center_.x(), center_.y(), center_.z());

        std::cout << "center before search: " << center << std::endl;
        octomap::OcTreeNode *node = octree_->search(x, y, z);
        std::cout << "center after search: " << center << std::endl;

        std::cout << "Searching worked!" << std::endl;

        if (node != nullptr && octree_->isNodeOccupied(node)) {
            std::cout << "node was occupied!" << std::endl;
            return false;
        }
        
        octomap::point3d target(x, y, z);
        octomap::point3d hit;

        std::cout << "center before castRay: " << center << std::endl;
        bool collision = octree_->castRay(center, target - center, hit, true, (target - center).norm() - 0.01);
        
        std::cout << "center after castRay: " << center << std::endl;

        std::cout << "CastRay worked!" << std::endl;
        return !collision;
    }


    ob::OptimizationObjectivePtr getPathLengthObjWithCostToGo(const ob::SpaceInformationPtr){
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        return obj;
    }

    Bounds bounds_;
    rclcpp::Logger logger_;
    ob::SpaceInformationPtr si;
    std::shared_ptr<ob::RealVectorStateSpace> space;
    ob::ProblemDefinitionPtr pdef;
};

