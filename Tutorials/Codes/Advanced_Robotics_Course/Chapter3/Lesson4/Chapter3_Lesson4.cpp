#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Simple axis-aligned rectangular obstacle
struct Rect
{
    double xmin, ymin, xmax, ymax;
};

bool isStateValid(const ob::State *state)
{
    const auto *s =
        state->as<ob::RealVectorStateSpace::StateType>();
    double x = (*s)[0];
    double y = (*s)[1];

    // example: one obstacle
    Rect obs{0.3, 0.3, 0.7, 0.4};
    bool inside =
        (x >= obs.xmin && x <= obs.xmax &&
         y >= obs.ymin && y <= obs.ymax);
    return !inside;
}

int main()
{
    auto space =
        std::make_shared<ob::RealVectorStateSpace>(2);
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0);
    bounds.setHigh(0, 1.0);
    bounds.setLow(1, 0.0);
    bounds.setHigh(1, 1.0);
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(&isStateValid);

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = 0.1; start[1] = 0.1;
    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal[0] = 0.9; goal[1] = 0.9;
    ss.setStartAndGoalStates(start, goal, 0.05);

    auto planner =
        std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    // optionally tune range and rewiring factor
    planner->setRange(0.1);
    ss.setPlanner(planner);

    ob::PlannerStatus solved =
        ss.solve(1.0); // 1 second

    if (solved)
    {
        auto path = ss.getSolutionPath();
        path.printAsMatrix(std::cout);
    }
    return 0;
}
      
