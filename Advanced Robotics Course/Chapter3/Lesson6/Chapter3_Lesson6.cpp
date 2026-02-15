#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/PRM.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

struct Obstacles
{
    // store meshes or primitives here
};

bool isStateValid(const ob::State *state, const Obstacles &obs)
{
    const auto *q = state->as<ob::RealVectorStateSpace::StateType>();
    // Convert OMPL state to joint vector
    std::array<double, 6> q_vec;
    for (std::size_t i = 0; i < 6; ++i)
        q_vec[i] = (*q)[i];

    // TODO: forward kinematics and collision checking for q_vec vs obs
    // return false if in collision
    return true;
}

og::SimpleSetup makeRRTStarPlanner(const Obstacles &obs,
                                   const std::array<double, 6> &qMin,
                                   const std::array<double, 6> &qMax)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::RealVectorBounds bounds(6);
    for (std::size_t i = 0; i < 6; ++i)
    {
        bounds.setLow(i, qMin[i]);
        bounds.setHigh(i, qMax[i]);
    }
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(
        [&obs](const ob::State *state)
        {
            return isStateValid(state, obs);
        });

    auto planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);
    return ss;
}

og::SimpleSetup makePRMPlanner(const Obstacles &obs,
                               const std::array<double, 6> &qMin,
                               const std::array<double, 6> &qMax)
{
    auto space = std::make_shared<ob::RealVectorStateSpace>(6);
    ob::RealVectorBounds bounds(6);
    for (std::size_t i = 0; i < 6; ++i)
    {
        bounds.setLow(i, qMin[i]);
        bounds.setHigh(i, qMax[i]);
    }
    space->setBounds(bounds);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(
        [&obs](const ob::State *state)
        {
            return isStateValid(state, obs);
        });

    auto planner = std::make_shared<og::PRM>(ss.getSpaceInformation());
    ss.setPlanner(planner);
    return ss;
}

int main()
{
    Obstacles obstacles;
    std::array<double, 6> qMin{{-3.14, -3.14, -3.14, -3.14, -3.14, -3.14}};
    std::array<double, 6> qMax{{ 3.14,  3.14,  3.14,  3.14,  3.14,  3.14}};

    auto ss = makeRRTStarPlanner(obstacles, qMin, qMax);

    ob::ScopedState<> start(ss.getStateSpace());
    ob::ScopedState<> goal(ss.getStateSpace());
    for (std::size_t i = 0; i < 6; ++i)
    {
        start[i] = 0.0;
        goal[i] = 1.0; // example
    }
    ss.setStartAndGoalStates(start, goal);

    ob::PlannerStatus solved = ss.solve(5.0); // seconds
    if (solved)
    {
        og::PathGeometric path = ss.getSolutionPath();
        path.interpolate();
        path.printAsMatrix(std::cout);
    }
    return 0;
}
      
