#include <pinocchio/algorithm/jacobian.hpp>

pinocchio::Model model;
pinocchio::buildModels::humanoidRandom(model);
pinocchio::Data data(model);

Eigen::VectorXd q = pinocchio::randomConfiguration(model);
Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);

// forward kinematics
pinocchio::forwardKinematics(model, data, q, v);

// Jacobian of a given frame
pinocchio::FrameIndex fid = model.getFrameId("left_foot");
Eigen::Matrix<double,6,Eigen::Dynamic> J6;
pinocchio::computeFrameJacobian(model, data, q, fid,
                                pinocchio::LOCAL_WORLD_ALIGNED, J6);
      
