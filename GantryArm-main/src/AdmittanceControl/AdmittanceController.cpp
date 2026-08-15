#include "AdmittanceController.h"
#include <algorithm>  // for std::clamp
#include <cstdio>
#include <stdexcept>

AdmittanceController::AdmittanceController(const IntegralParams& params,
    bool use_integral)
    : params_(params),
    use_integral_(use_integral)
{
    if (!params_.validate()) {
        throw std::invalid_argument("Invalid IntegralParams");
    }
    x_.setZero();
    v_.setZero();
    integral_.setZero();
    F_desired_.setZero();
}

Point6D AdmittanceController::update(const Eigen::Matrix<double, 6, 1>& F_measured) {
    return update(F_measured, F_desired_);
}

Point6D AdmittanceController::update(const Eigen::Matrix<double, 6, 1>& F_measured,
    const Eigen::Matrix<double, 6, 1>& F_desired)
{
    if (use_integral_) {
        updateIntegral(F_measured, F_desired);
    }

    Eigen::Matrix<double, 6, 1> a = computeAcceleration(F_measured, F_desired);

    v_ += a * params_.dt;
    x_ += v_ * params_.dt;

    return Point6D(x_(0)*1000, x_(1)*1000, x_(2)*1000, x_(3), x_(4), x_(5));
}

Point6D AdmittanceController::updateZ(double F_z_measured)
{
    Eigen::Matrix<double, 6, 1> F_m = F_desired_;
    F_m(2) = F_z_measured;
    return update(F_m, F_desired_);
}

void AdmittanceController::setDesiredWrench(const Eigen::Matrix<double, 6, 1>& F_d)
{
    F_desired_ = F_d;
}

void AdmittanceController::setDesiredForce(int dof_index, double value)
{
    if (dof_index < 0 || dof_index >= 6) return;
    F_desired_(dof_index) = value;
}

Eigen::Matrix<double, 6, 1> AdmittanceController::getDesiredWrench() const
{
    return F_desired_;
}

Point6D AdmittanceController::getPositionOffset() const
{
    return Point6D(x_(0), x_(1), x_(2), x_(3), x_(4), x_(5));
}

Eigen::Matrix<double, 6, 1> AdmittanceController::getVelocity() const
{
    return v_;
}

Eigen::Matrix<double, 6, 1> AdmittanceController::getIntegral() const
{
    return integral_;
}

void AdmittanceController::reset()
{
    x_.setZero();
    v_.setZero();
    integral_.setZero();
}

void AdmittanceController::resetIntegral()
{
    integral_.setZero();
}

void AdmittanceController::enableIntegral(bool enable)
{
    use_integral_ = enable;
}

void AdmittanceController::setParams(const IntegralParams& params)
{
    if (!params.validate()) {
        throw std::invalid_argument("Invalid IntegralParams");
    }
    params_ = params;
}

IntegralParams AdmittanceController::getParams() const
{
    return params_;
}

void AdmittanceController::printInfo() const
{
    printf("\n===== Admittance Controller Info =====\n");
    printf("Integral enabled: %s\n", use_integral_ ? "Yes" : "No");
    printf("Desired wrench: [");
    for (int i = 0; i < 6; ++i) {
        printf("%.2f", F_desired_(i));
        if (i < 5) printf(", ");
    }
    printf("]\n");
    params_.print();
    printf("======================================\n");
}

// 私有函数

// Eigen::Matrix<double, 6, 1> AdmittanceController::computeAcceleration(
//     const Eigen::Matrix<double, 6, 1>& F_m,
//     const Eigen::Matrix<double, 6, 1>& F_d)
// {
//     Eigen::Matrix<double, 6, 1> force_error = F_m - F_d;

//     Eigen::Matrix<double, 6, 1> integral_comp = Eigen::Matrix<double, 6, 1>::Zero();
//     if (use_integral_) {
//         integral_comp = params_.eta.cwiseQuotient(params_.k_e).cwiseProduct(integral_);
//     }

//     Eigen::Matrix<double, 6, 1> a = (force_error + integral_comp
//         - params_.damping.cwiseProduct(v_)
//         - params_.stiffness.cwiseProduct(x_))
//         .cwiseQuotient(params_.mass);

//     return a;
// }

Eigen::Matrix<double, 6, 1> AdmittanceController::computeAcceleration(
    const Eigen::Matrix<double, 6, 1>& F_m,
    const Eigen::Matrix<double, 6, 1>& F_d)
{
    Eigen::Matrix<double, 6, 1> force_error = F_m - F_d;

    Eigen::Matrix<double, 6, 1> integral_comp = Eigen::Matrix<double, 6, 1>::Zero();
    if (use_integral_) {
        integral_comp = params_.eta.cwiseQuotient(params_.k_e).cwiseProduct(integral_);
    }

    // 计算非线性刚度，示意用一个简单的线性钳制模型:
    Eigen::Matrix<double, 6, 1> nonlinear_stiffness;
    for (int i = 0; i < 6; ++i) {
        double f = force_error(i);
        double k_base = params_.stiffness(i);
        // 负力时刚度降低50%作为示例
        nonlinear_stiffness(i) = (f < -40) ? (0.1 * k_base) : k_base;
    }

    Eigen::Matrix<double, 6, 1> a = (force_error + integral_comp
                                     - params_.damping.cwiseProduct(v_)
                                     - nonlinear_stiffness.cwiseProduct(x_))
                                        .cwiseQuotient(params_.mass);

    return a;
}

void AdmittanceController::updateIntegral(
    const Eigen::Matrix<double, 6, 1>& F_m,
    const Eigen::Matrix<double, 6, 1>& F_d)
{
    Eigen::Matrix<double, 6, 1> force_error = F_m - F_d;

    Eigen::Matrix<double, 6, 1> int_dot = force_error - params_.integral_decay.cwiseProduct(integral_);

    integral_ += int_dot * params_.dt;

    // 限幅
    limitIntegral();
}

void AdmittanceController::limitIntegral()
{
    for (int i = 0; i < 6; ++i) {
        integral_(i) = std::clamp(integral_(i),
            -params_.integral_max(i),
            params_.integral_max(i));
    }
}
