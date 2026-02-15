#include <vector>
#include <array>
#include <cmath>
#include <fcl/fcl.h>

// Simple alias for a configuration (n-DOF)
using Config = std::vector<double>;

struct CSpaceCell {
    Config center;
    double radius;     // radius in configuration metric
    double dist_ws;    // workspace distance d_W(center)
    enum class Label { Unknown, Free, Colliding } label;
};

class CSpaceApproximator {
public:
    CSpaceApproximator(double Lw,
                       const std::vector<std::shared_ptr<fcl::CollisionObjectd> >& robot_links,
                       const std::vector<std::shared_ptr<fcl::CollisionObjectd> >& obstacles)
        : Lw_(Lw), robot_links_(robot_links), obstacles_(obstacles) {}

    void classifyCell(CSpaceCell& cell) {
        // 1. Place robot according to cell.center
        forwardKinematicsToFCL(cell.center);

        // 2. Query minimal distance across all link-obstacle pairs
        double dmin = queryWorkspaceDistance();

        cell.dist_ws = dmin;

        if (dmin >  Lw_ * cell.radius) {
            cell.label = CSpaceCell::Label::Free;
        } else if (dmin < -Lw_ * cell.radius) {
            cell.label = CSpaceCell::Label::Colliding;
        } else {
            cell.label = CSpaceCell::Label::Unknown;
        }
    }

private:
    double Lw_;
    std::vector<std::shared_ptr<fcl::CollisionObjectd> > robot_links_;
    std::vector<std::shared_ptr<fcl::CollisionObjectd> > obstacles_;

    void forwardKinematicsToFCL(const Config& q) {
        // Update the pose of each robot_links_[i] based on the joint configuration q.
        // This is a standard kinematic update using your favorite math library (Eigen, etc.).
        // ...
    }

    double queryWorkspaceDistance() {
        fcl::DistanceRequestd req;
        fcl::DistanceResultd res;
        double dmin = std::numeric_limits<double>::infinity();

        for (const auto& link : robot_links_) {
            for (const auto& obs : obstacles_) {
                res.clear();
                fcl::distance(link.get(), obs.get(), req, res);
                dmin = std::min(dmin, res.min_distance);
            }
        }
        return dmin;
    }
};
      
