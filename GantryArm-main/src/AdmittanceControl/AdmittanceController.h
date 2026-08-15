#pragma once
#include <Eigen/Dense>
#include <cstdio>
#include <cmath>
#include "robot_types.h"

/**
 * @brief 导纳控制参数，融合积分补偿参数
 *
 * 参数说明及调参建议：
 *
 * 虚拟质量（mass）[kg / kg·m²]
 *   - 表示系统在各自由度上的惯性大小。
 *   - 数值越大，系统响应越平稳但动作更慢，惯性较大。
 *   - 调整建议：根据机械臂实际负载或惯量合理设置，通常平动1~5kg，旋转0.1~1kg·m²。
 *
 * 虚拟阻尼（damping）[N·s/m 或 N·m·s/rad]
 *   - 控制系统阻尼特性，影响振荡和响应速度。
 *   - 理想设置约为临界阻尼：B ≈ 2 * sqrt(M * K)，保证无振荡的快速响应。
 *   - 调整建议：增大阻尼减少振荡，减小阻尼提高灵敏度但可能振荡。
 *
 * 虚拟刚度（stiffness）[N/m 或 N·m/rad]
 *   - 表示系统弹性，刚度越大系统行为越“硬”，位姿偏移小，柔顺度低。
 *   - 调整建议：任务要求柔顺时减小刚度，精度和稳定要求时增大。
 *
 * 积分参数：
 *   - eta（积分增益分子）与 k_e（积分增益分母）
 *     积分项实际增益为 η / k_e。
 *   - 整体用于消除稳态误差，克服系统摩擦等非线性影响。
 *   - 调整建议：选择中等η/k_e（如3~5），过大易引入振荡和积分饱和。
 *
 * 积分限幅（integral_max）
 *   - 限制积分量最大绝对值，防止积分风暴。
 *   - 调整建议：依据系统可接受最大稳态误差范围设置。
 *
 * 积分衰减（integral_decay）
 *   - 积分项的递减速率，避免积分项无限增长。
 *   - 调整建议：通常为0.05~0.2之间，适当调减改善系统稳定性。
 *
 * 采样周期（dt）[秒]
 *   - 控制指令更新周期，数值越小响应越细腻。
 *   - 常用1ms（0.001s）为机械臂控制频率。
 */

struct IntegralParams {
	Eigen::Matrix<double, 6, 1> mass;          ///< 虚拟质量
	Eigen::Matrix<double, 6, 1> damping;       ///< 虚拟阻尼
	Eigen::Matrix<double, 6, 1> stiffness;     ///< 虚拟刚度

	Eigen::Matrix<double, 6, 1> eta;            ///< 积分增益分子η
	Eigen::Matrix<double, 6, 1> k_e;            ///< 积分增益分母k_e
	Eigen::Matrix<double, 6, 1> integral_max;   ///< 积分限幅
	Eigen::Matrix<double, 6, 1> integral_decay; ///< 积分衰减

	double dt;        ///< 控制采样周期（秒）

	IntegralParams()
		: dt(0.001)
	{
		// 默认参数示例：
		// 平动质量1kg，阻尼10，刚度100，对应自然频率10 rad/s，阻尼约临界
		mass.head<3>().setConstant(1.0);
		damping.head<3>().setConstant(10.0);
		stiffness.head<3>().setConstant(100.0);

		// 旋转质量0.1kg·m²，阻尼1，刚度10，符合机械臂旋转特性
		mass.tail<3>().setConstant(0.1);
		damping.tail<3>().setConstant(1.0);
		stiffness.tail<3>().setConstant(10.0);

		// 积分参数：η=5.0，k_e=1.0，积分限幅50，积分衰减0.1
		eta.setConstant(5.0);
		k_e.setConstant(1.0);
		integral_max.setConstant(50.0);
		integral_decay.setConstant(0.1);
	}

	// 简化构造，统一平动和平转参数
	IntegralParams(double m_t, double d_t, double k_t,
		double m_r, double d_r, double k_r,
		double eta_val = 5.0, double ke_val = 1.0,
		double int_max = 50.0, double decay = 0.1,
		double dt_val = 0.001)
		: dt(dt_val)
	{
		mass.head<3>().setConstant(m_t);
		damping.head<3>().setConstant(d_t);
		stiffness.head<3>().setConstant(k_t);

		mass.tail<3>().setConstant(m_r);
		damping.tail<3>().setConstant(d_r);
		stiffness.tail<3>().setConstant(k_r);

		eta.setConstant(eta_val);
		k_e.setConstant(ke_val);
		integral_max.setConstant(int_max);
		integral_decay.setConstant(decay);
	}
	bool validate() const {
		for (int i = 0; i < 6; ++i) {
			if (mass(i) <= 0 || damping(i) < 0 || stiffness(i) < 0)
				return false;
			if (eta(i) < 0 || k_e(i) <= 0 || integral_max(i) < 0 || integral_decay(i) < 0)
				return false;
		}
		return dt > 0;
	}

	void print() const {
		printf("\nIntegralParams:\n");
		printf(" dt = %.4f s\n", dt);
		printf(" DOF |   M    |   B    |   K    |  η   |  k_e |  IntMax | Decay\n");
		printf("-----|--------|--------|--------|------|------|---------|-------\n");
		const char* names[]{ "X","Y","Z","Rx","Ry","Rz" };
		for (int i = 0; i < 6; ++i) {
			printf(" %-3s | %6.2f | %6.2f | %6.1f | %4.1f | %4.1f | %7.1f | %.3f\n",
				names[i], mass(i), damping(i), stiffness(i),
				eta(i), k_e(i), integral_max(i), integral_decay(i));
		}
	}
};


/**
 * @brief 6自由度导纳控制器，简化设计，无掩码，积分可开关
 */
class AdmittanceController {
public:
	explicit AdmittanceController(const IntegralParams& params, bool use_integral = false);

	// 更新，内置期望力
	Point6D update(const Eigen::Matrix<double, 6, 1>& F_measured);

	// 显式更新，传期望力
	Point6D update(const Eigen::Matrix<double, 6, 1>& F_measured,
		const Eigen::Matrix<double, 6, 1>& F_desired);

	// 只用于Z轴力控制，快速接口
	Point6D updateZ(double F_z_measured);

	// 期望力设置和查询
	void setDesiredWrench(const Eigen::Matrix<double, 6, 1>& F_d);
	void setDesiredForce(int dof_index, double value);
	Eigen::Matrix<double, 6, 1> getDesiredWrench() const;

	// 状态查询
	Point6D getPositionOffset() const;
	Eigen::Matrix<double, 6, 1> getVelocity() const;
	Eigen::Matrix<double, 6, 1> getIntegral() const;

	// 控制器重置
	void reset();
	void resetIntegral();

	// 设置积分使用开关
	void enableIntegral(bool enable);

	// 参数更新及打印
	void setParams(const IntegralParams& params);
	IntegralParams getParams() const;
	void printInfo() const;

private:
	IntegralParams params_;

	Eigen::Matrix<double, 6, 1> x_;         // 位姿偏移
	Eigen::Matrix<double, 6, 1> v_;         // 速度
	Eigen::Matrix<double, 6, 1> integral_;  // 积分项
	Eigen::Matrix<double, 6, 1> F_desired_; // 期望力

	bool use_integral_;

	Eigen::Matrix<double, 6, 1> computeAcceleration(const Eigen::Matrix<double, 6, 1>& F_m,
		const Eigen::Matrix<double, 6, 1>& F_d);
	void updateIntegral(const Eigen::Matrix<double, 6, 1>& F_m,
		const Eigen::Matrix<double, 6, 1>& F_d);
	void limitIntegral();
};