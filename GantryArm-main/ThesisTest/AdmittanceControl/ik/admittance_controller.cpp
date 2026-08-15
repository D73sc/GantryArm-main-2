//#include "admittance_controller.h"
//#include <algorithm>
//
//AdmittanceController::AdmittanceController(
//	const IntegralParams& params,
//	const DOFMask& mask,
//	bool use_integral)
//	: params_(params)
//	, mask_(mask)
//	, use_integral_(use_integral)
//	, limit_integral_(true)
//	, decay_integral_(true)
//{
//	if (!params_.validate()) {
//		throw std::invalid_argument("Invalid parameters");
//	}
//
//	x_.setZero();
//	v_.setZero();
//	integral_.setZero();
//	F_desired_.setZero();
//}
//
//Point6D AdmittanceController::update(const Vector6d& F_measured) {
//	return update(F_measured, F_desired_);
//}
//
//Point6D AdmittanceController::update(const Vector6d& F_measured, const Vector6d& F_desired) {
//	// 1. 更新积分项
//	if (use_integral_) {
//		updateIntegral(F_measured, F_desired);
//	}
//
//	// 2. 计算加速度
//	Vector6d a = computeAcceleration(F_measured, F_desired);
//
//	// 3. 欧拉积分
//	v_ += a * params_.dt;
//	x_ += v_ * params_.dt;
//
//	// 4. 返回Point6D格式
//	return toPoint6D(applyMask(x_));
//}
//
//Point6D AdmittanceController::updateZ(double F_z_measured) {
//	Vector6d F_m = F_desired_;
//	F_m(2) = F_z_measured;
//	return update(F_m, F_desired_);
//}
//
//Point6D AdmittanceController::getPositionOffset() const {
//	return toPoint6D(x_);
//}
//
//void AdmittanceController::reset() {
//	x_.setZero();
//	v_.setZero();
//	integral_.setZero();
//}
//
//void AdmittanceController::resetIntegral() {
//	integral_.setZero();
//}
//
//void AdmittanceController::setParams(const IntegralParams& params) {
//	if (!params.validate()) {
//		throw std::invalid_argument("Invalid parameters");
//	}
//	params_ = params;
//}
//
//void AdmittanceController::printInfo() const {
//	printf("\n========================================\n");
//	printf("Admittance Controller Info\n");
//	printf("========================================\n");
//
//	printf("\nEnabled DOFs: ");
//	const char* names[] = { "X", "Y", "Z", "Rx", "Ry", "Rz" };
//	for (int i = 0; i < 6; ++i) {
//		if (mask_.isEnabled(i)) printf("%s ", names[i]);
//	}
//	printf("(%d/6)\n", mask_.count());
//
//	printf("Integral: %s\n", use_integral_ ? "ON" : "OFF");
//	printf("Desired Wrench: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n",
//		F_desired_(0), F_desired_(1), F_desired_(2),
//		F_desired_(3), F_desired_(4), F_desired_(5));
//
//	params_.print();
//	printf("========================================\n");
//}
//
//// ========== 私有函数 ==========
//
//Vector6d AdmittanceController::computeAcceleration(
//	const Vector6d& F_m, const Vector6d& F_d)
//{
//	Vector6d force_error = F_m - F_d;
//
//	Vector6d integral_comp = Vector6d::Zero();
//	if (use_integral_) {
//		integral_comp = params_.eta.cwiseQuotient(params_.k_e).cwiseProduct(integral_);
//	}
//
//	Vector6d a = (force_error + integral_comp
//		- params_.damping.cwiseProduct(v_)
//		- params_.stiffness.cwiseProduct(x_))
//		.cwiseQuotient(params_.mass);
//
//	return applyMask(a);
//}
//
//void AdmittanceController::updateIntegral(const Vector6d& F_m, const Vector6d& F_d) {
//	Vector6d force_error = F_m - F_d;
//	Vector6d int_dot = force_error;
//
//	if (decay_integral_) {
//		int_dot -= params_.integral_decay.cwiseProduct(integral_);
//	}
//
//	integral_ += applyMask(int_dot) * params_.dt;
//
//	if (limit_integral_) {
//		limitIntegral();
//	}
//}
//
//void AdmittanceController::limitIntegral() {
//	for (int i = 0; i < 6; ++i) {
//		if (mask_.isEnabled(i)) {
//			integral_(i) = std::clamp(integral_(i),
//				-params_.integral_max(i),
//				params_.integral_max(i));
//		}
//	}
//}
//
//Vector6d AdmittanceController::applyMask(const Vector6d& vec) const {
//	Vector6d result = vec;
//	for (int i = 0; i < 6; ++i) {
//		if (!mask_.isEnabled(i)) {
//			result(i) = 0.0;
//		}
//	}
//	return result;
//}
//
//Point6D AdmittanceController::toPoint6D(const Vector6d& vec) {
//	return Point6D(vec(0), vec(1), vec(2), vec(3), vec(4), vec(5));
//}
//
//Vector6d AdmittanceController::toVector6d(const Point6D& point) {
//	Vector6d vec;
//	vec << point.x, point.y, point.z, point.rx, point.ry, point.rz;
//	return vec;
//}
//
