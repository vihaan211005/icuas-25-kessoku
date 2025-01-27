#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <octomap/OcTree.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Define a validity checker using the OctoMap
class OctoMapValidityChecker : public ob::StateValidityChecker {
public:
    OctoMapValidityChecker(const ob::SpaceInformationPtr &si, const octomap::OcTree &octree)
        : ob::StateValidityChecker(si), octree_(octree) {}

    // Override the pure virtual function
    bool isValid(const ob::State *state) const override {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        double z = realState->values[2];

        // Custom validity logic using the OctoMap
        octomap::OcTreeNode *node = octree_.search(x, y, z);
        return (node == nullptr || octree_.isNodeOccupied(node) == false);
    }

    // Custom extended validity checking method
    bool isValid(const ob::State *state, double x_target, double y_target, double z_target, bool check_line_of_sight) const {
        if (!isValid(state)) {
            return false;
        }

        // Add custom logic for checking line-of-sight or other criteria if needed
        // Example: Return true only if within a specific range of the target
        return true;
    }

private:
    const octomap::OcTree &octree_;
};


int main() {
    // Load the OctoMap file
    std::string octomapFile = "city_1.binvox.bt";
    octomap::OcTree octree(octomapFile);

    if (octree.getTreeDepth() == 0) {
        std::cerr << "Failed to load OctoMap!" << std::endl;
        return -1;
    }

    std::cout << "OctoMap loaded successfully!" << std::endl;

    // Define the state space (3D space)
    auto space = std::make_shared<ob::RealVectorStateSpace>(3);

    // Set bounds for the space (assuming [-10, 10] for simplicity; adjust to your map size)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-10);
    bounds.setHigh(10);
    space->setBounds(bounds);

    // Create the space information object
    auto si = std::make_shared<ob::SpaceInformation>(space);

    // Set the state validity checker
    auto validityChecker = std::make_shared<OctoMapValidityChecker>(si, octree);
    si->setStateValidityChecker(validityChecker);
    si->setup();
    si->setup();

    // Define start and goal states
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = -5;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = -5;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0;

    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 5;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 5;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 0;

    // Create the motion planning problem
    og::SimpleSetup ss(si);
    ss.setStartAndGoalStates(start, goal);

    // Set the planner (RRT in this case)
    auto planner = std::make_shared<og::RRT>(si);
    ss.setPlanner(planner);

    // Solve the problem
    ob::PlannerStatus solved = ss.solve(1.0); // 1-second timeout

    if (solved) {
        std::cout << "Found a solution!" << std::endl;

        // Print the solution path
        ss.simplifySolution();
        og::PathGeometric path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
    } else {
        std::cerr << "No solution found." << std::endl;
    }

    return 0;
}
