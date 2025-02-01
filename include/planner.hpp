#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <octomap/OcTree.h>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

class OctoMapValidityChecker : public ob::StateValidityChecker {
public:
    OctoMapValidityChecker(ob::SpaceInformationPtr &si, octomap::OcTree* octree)
        : ob::StateValidityChecker(si), octree_(octree) {}

    bool isValid(const ob::State *state) const override {
        const auto *realState = state->as<ob::RealVectorStateSpace::StateType>();
        double x = realState->values[0];
        double y = realState->values[1];
        double z = realState->values[2];

        octomap::OcTreeNode *node = octree_->search(x, y, z);
        return (node == nullptr || octree_->isNodeOccupied(node) == false);
    }

    bool isValid(const ob::State *state, double x_target, double y_target, double z_target, bool check_line_of_sight) const {
        if (!isValid(state)) {
            return false;
        }

        // Add custom logic for checking line-of-sight or other criteria if needed
        // Example: Return true only if within a specific range of the target
        return true;
    }

private:
    octomap::OcTree* octree_;
};