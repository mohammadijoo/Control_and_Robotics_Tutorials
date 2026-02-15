#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/fmt/FMT.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

bool isStateValid(const ob::State *state) {
    const auto *s = state->as<ob::RealVectorStateSpace::StateType>();
    double x = (*s)[0];
    double y = (*s)[1];
    // TODO: collision check in workspace
    (void)x; (void)y;
    return true;
}

int main() {
    // 1. Define a 2D state space
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, -5.0);
    bounds.setHigh(0,  5.0);
    bounds.setLow(1, -5.0);
    bounds.setHigh(1,  5.0);
    space->setBounds(bounds);

    // 2. SimpleSetup wraps common boilerplate
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(&isStateValid);

    ob::ScopedState<> start(space), goal(space);
    start[0] = -4.0; start[1] = -4.0;
    goal[0]  =  4.0; goal[1]  =  4.0;
    ss.setStartAndGoalStates(start, goal);

    // 3. Choose planner: BIT* or FMT*
    auto si = ss.getSpaceInformation();
    auto planner = std::make_shared<og::BITstar>(si);
    // For FMT*, use: auto planner = std::make_shared<og::FMT>(si);

    ss.setPlanner(planner);

    // 4. Solve with time budget
    ob::PlannerStatus solved = ss.solve(5.0);
    if (solved) {
        ss.simplifySolution();
        auto path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
    }
    return 0;
}
      
