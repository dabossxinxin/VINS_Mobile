
#ifndef POSE_LOCAL_PARAMETERIZATION_HPP
#define POSE_LOCAL_PARAMETERIZATION_HPP

#include <ceres/ceres.h>

class PoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const;
    virtual bool ComputeJacobian(const double *x, double *jacobian) const;
    virtual int GlobalSize() const { return 7; };
    virtual int LocalSize() const { return 6; };
};

#endif /* pose_local_parameterization_hpp */
