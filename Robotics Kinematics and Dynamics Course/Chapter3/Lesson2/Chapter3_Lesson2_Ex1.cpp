Eigen::AngleAxisd aa(theta, axis.normalized());
Matrix3d R = aa.toRotationMatrix();
      
