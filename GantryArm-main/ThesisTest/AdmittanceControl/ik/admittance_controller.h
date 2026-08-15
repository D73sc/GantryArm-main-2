///**
// * @file admittance_controller.h
// * @brief 6自由度导纳控制器（配合robot_types使用）
// *
// * 使用方法：
// *   1. 创建控制器并设置参数
// *   2. 设置期望力（固定值）
// *   3. 每个周期调用update()获取位姿偏移
// *   4. 将偏移加到当前位姿，调用逆运动学
// *
// * 动力学方程：
// *   传统: M*ẍ + B*ẋ + K*x = F_measured - F_desired
// *   积分: M*ẍ + B*ẋ + K*x = (F_m - F_d) + (η/k_e)∫(F_m - F_d)dt
// */
//
//#ifndef ADMITTANCE_CONTROLLER_H
//#define ADMITTANCE_CONTROLLER_H
//
//#include "types.h"
//#include "robot_types.h"  // 使用你的Point6D和RobotPose
//
//class AdmittanceController {
//public:
//	/**
//	 * @brief 构造函数
//	 * @param params 控制参数
//	 * @param mask 启用的自由度掩码
//	 * @param use_integral 是否启用积分补偿
//	 */
//	explicit AdmittanceController(
//		const IntegralParams& params,
//		const DOFMask& mask = DOFMask::all(),
//		bool use_integral = false
//	);
//
//	~AdmittanceController() = default;
//
//	// ========== 主要接口 ==========
//
//	/**
//	 * @brief 更新控制器（使用内部期望力）
//	 * @param F_measured 测量的力/力矩 [Fx,Fy,Fz,Tx,Ty,Tz]
//	 * @return 位姿偏移量（Point6D格式）
//	 */
//	Point6D update(const Vector6d& F_measured);
//
//	/**
//	 * @brief 更新控制器（显式传入期望力）
//	 * @param F_measured 测量的力/力矩
//	 * @param F_desired 期望力/力矩
//	 * @return 位姿偏移量
//	 */
//	Point6D update(const Vector6d& F_measured, const Vector6d& F_desired);
//
//	/**
//	 * @brief 更新（只传入单个自由度的力，常用于Z轴力控制）
//	 * @param F_z_measured Z轴测量力
//	 * @return 位姿偏移量
//	 */
//	Point6D updateZ(double F_z_measured);
//
//	// ========== 期望力设置 ==========
//
//	void setDesiredWrench(const Vector6d& F_d) { F_desired_ = F_d; }
//	void setDesiredForce(DOF dof, double value) {
//		F_desired_(static_cast<int>(dof)) = value;
//	}
//	Vector6d getDesiredWrench() const { return F_desired_; }
//
//	// ========== 状态查询 ==========
//	Point6D getPositionOffset() const;  // 获取当前位姿偏移
//	Vector6d getVelocity() const { return v_; }
//	Vector6d getIntegral() const { return integral_; }
//
//	// ========== 控制 ==========
//	void reset();
//	void resetIntegral();
//	void setMask(const DOFMask& mask) { mask_ = mask; }
//	void enableIntegral(bool enable) { use_integral_ = enable; }
//	void enableIntegralLimit(bool enable) { limit_integral_ = enable; }
//	void enableIntegralDecay(bool enable) { decay_integral_ = enable; }
//
//	IntegralParams getParams() const { return params_; }
//	void setParams(const IntegralParams& params);
//
//	void printInfo() const;
//
//private:
//	IntegralParams params_;
//	DOFMask mask_;
//
//	Vector6d x_;          // 位姿偏移 [x,y,z,rx,ry,rz]
//	Vector6d v_;          // 速度
//	Vector6d integral_;   // 积分项
//	Vector6d F_desired_;  // 期望力
//
//	bool use_integral_;
//	bool limit_integral_;
//	bool decay_integral_;
//
//	Vector6d computeAcceleration(const Vector6d& F_m, const Vector6d& F_d);
//	void updateIntegral(const Vector6d& F_m, const Vector6d& F_d);
//	void limitIntegral();
//	Vector6d applyMask(const Vector6d& vec) const;
//
//	// 工具函数：Vector6d <-> Point6D
//	static Point6D toPoint6D(const Vector6d& vec);
//	static Vector6d toVector6d(const Point6D& point);
//};
//
//
//#endif