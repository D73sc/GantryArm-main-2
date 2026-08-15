///**
// * @file types.h
// * @brief 导纳控制器类型定义（配合robot_types使用）
// *
// * 说明：
// *   - 复用robot_types中的Point6D
// *   - 导纳控制器输出位姿偏移量（Point6D格式）
// *   - 用户自行调用逆运动学求解关节角
// *
// * @date 2024
// */
//
//#ifndef ADMITTANCE_TYPES_H
//#define ADMITTANCE_TYPES_H
//
//#include <Eigen/Dense>
//#include <array>
//#include <iostream>
//#include <cmath>
//#include <stdexcept>
//
//
// // 类型别名
//using Vector6d = Eigen::Matrix<double, 6, 1>;
//using Vector3d = Eigen::Vector3d;
//
///**
// * @brief 笛卡尔空间自由度索引
// */
//enum class DOF : int {
//	X = 0,   // X平移
//	Y = 1,   // Y平移
//	Z = 2,   // Z平移
//	RX = 3,   // 绕X旋转
//	RY = 4,   // 绕Y旋转
//	RZ = 5    // 绕Z旋转
//};
//
///**
// * @brief 自由度控制掩码（选择启用哪些自由度）
// */
//struct DOFMask {
//	std::array<bool, 6> enabled;
//
//	DOFMask() { enabled.fill(false); }
//
//	// 预设掩码
//	static DOFMask all() {
//		DOFMask mask;
//		mask.enabled.fill(true);
//		return mask;
//	}
//
//	static DOFMask zOnly() {
//		DOFMask mask;
//		mask.enabled[2] = true;  // 只启用Z
//		return mask;
//	}
//
//	static DOFMask xyOnly() {
//		DOFMask mask;
//		mask.enabled[0] = mask.enabled[1] = true;
//		return mask;
//	}
//
//	static DOFMask xyzOnly() {
//		DOFMask mask;
//		mask.enabled[0] = mask.enabled[1] = mask.enabled[2] = true;
//		return mask;
//	}
//
//	void enable(DOF dof) {
//		enabled[static_cast<int>(dof)] = true;
//	}
//
//	void disable(DOF dof) {
//		enabled[static_cast<int>(dof)] = false;
//	}
//
//	bool isEnabled(DOF dof) const {
//		return enabled[static_cast<int>(dof)];
//	}
//
//	bool isEnabled(int idx) const {
//		return enabled[idx];
//	}
//
//	int count() const {
//		int cnt = 0;
//		for (bool e : enabled) if (e) cnt++;
//		return cnt;
//	}
//};
//
///**
// * @brief 导纳控制参数
// */
//struct AdmittanceParams {
//	Vector6d mass;        // 虚拟质量 [kg] 或 [kg·m²]
//	Vector6d damping;     // 虚拟阻尼 [N·s/m] 或 [N·m·s/rad]
//	Vector6d stiffness;   // 虚拟刚度 [N/m] 或 [N·m/rad]
//	double dt;            // 采样时间 [s]
//
//	AdmittanceParams() : dt(0.001) {
//		mass.setConstant(1.0);
//		damping.setConstant(10.0);
//		stiffness.setConstant(100.0);
//	}
//
//	AdmittanceParams(double m, double d, double k, double dt_val)
//		: dt(dt_val)
//	{
//		mass.setConstant(m);
//		damping.setConstant(d);
//		stiffness.setConstant(k);
//	}
//
//	// 分别设置平移和旋转参数
//	AdmittanceParams(double m_t, double d_t, double k_t,
//		double m_r, double d_r, double k_r, double dt_val)
//		: dt(dt_val)
//	{
//		mass.head<3>().setConstant(m_t);
//		damping.head<3>().setConstant(d_t);
//		stiffness.head<3>().setConstant(k_t);
//		mass.tail<3>().setConstant(m_r);
//		damping.tail<3>().setConstant(d_r);
//		stiffness.tail<3>().setConstant(k_r);
//	}
//
//	void setDOF(DOF dof, double m, double d, double k) {
//		int i = static_cast<int>(dof);
//		mass(i) = m;
//		damping(i) = d;
//		stiffness(i) = k;
//	}
//
//	double naturalFreq(DOF dof) const {
//		int i = static_cast<int>(dof);
//		return std::sqrt(stiffness(i) / mass(i));
//	}
//
//	double dampingRatio(DOF dof) const {
//		int i = static_cast<int>(dof);
//		return damping(i) / (2.0 * std::sqrt(mass(i) * stiffness(i)));
//	}
//
//	void print() const {
//		printf("\nAdmittance Parameters:\n");
//		printf("  dt = %.4f s\n", dt);
//		printf("  DOF |   M    |   B    |   K    |  ωn   |  ζ\n");
//		printf("  ----|--------|--------|--------|-------|------\n");
//		const char* names[] = { "X", "Y", "Z", "Rx", "Ry", "Rz" };
//		for (int i = 0; i < 6; ++i) {
//			double wn = std::sqrt(stiffness(i) / mass(i));
//			double zeta = damping(i) / (2.0 * std::sqrt(mass(i) * stiffness(i)));
//			printf("  %-3s | %6.2f | %6.2f | %6.1f | %5.2f | %.3f\n",
//				names[i], mass(i), damping(i), stiffness(i), wn, zeta);
//		}
//	}
//
//	bool validate() const {
//		for (int i = 0; i < 6; ++i) {
//			if (mass(i) <= 0 || damping(i) < 0 || stiffness(i) < 0) {
//				return false;
//			}
//		}
//		return dt > 0;
//	}
//};
//
///**
// * @brief 带积分补偿的参数
// */
//struct IntegralParams : public AdmittanceParams {
//	Vector6d eta;              // 积分增益分子
//	Vector6d k_e;              // 积分增益分母
//	Vector6d integral_max;     // 积分限幅
//	Vector6d integral_decay;   // 积分衰减
//
//	IntegralParams() : AdmittanceParams() {
//		eta.setConstant(5.0);
//		k_e.setConstant(1.0);
//		integral_max.setConstant(50.0);
//		integral_decay.setConstant(0.1);
//	}
//
//	IntegralParams(double m, double d, double k, double dt_val) :AdmittanceParams(m, d, k, dt_val)
//	{
//	}
//
//	IntegralParams(double m, double d, double k, double dt_val,
//		double eta_val, double ke_val = 1.0,
//		double int_max = 50.0, double decay = 0.1)
//		: AdmittanceParams(m, d, k, dt_val)
//	{
//		eta.setConstant(eta_val);
//		k_e.setConstant(ke_val);
//		integral_max.setConstant(int_max);
//		integral_decay.setConstant(decay);
//	}
//
//	void setIntegralDOF(DOF dof, double eta_val, double ke_val = 1.0,
//		double int_max = 50.0, double decay = 0.1) {
//		int i = static_cast<int>(dof);
//		eta(i) = eta_val;
//		k_e(i) = ke_val;
//		integral_max(i) = int_max;
//		integral_decay(i) = decay;
//	}
//
//	Vector6d integralGain() const {
//		return eta.cwiseQuotient(k_e);
//	}
//
//	void print() const {
//		AdmittanceParams::print();
//		printf("\n  Integral Parameters:\n");
//		printf("  DOF | η/k_e | Max  | Decay\n");
//		printf("  ----|-------|------|------\n");
//		const char* names[] = { "X", "Y", "Z", "Rx", "Ry", "Rz" };
//		for (int i = 0; i < 6; ++i) {
//			printf("  %-3s | %5.2f | %4.1f | %.3f\n",
//				names[i], eta(i) / k_e(i), integral_max(i), integral_decay(i));
//		}
//	}
//};
//
//
//#endif // ADMITTANCE_TYPES_H