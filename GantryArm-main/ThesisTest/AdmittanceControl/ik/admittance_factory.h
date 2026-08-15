///**
// * @file admittance_factory.h
// * @brief 导纳控制器工厂
// */
//
//#ifndef ADMITTANCE_FACTORY_H
//#define ADMITTANCE_FACTORY_H
//
//#include "admittance_controller.h"
//#include <memory>
//
//namespace admittance {
//
//    class Factory {
//    public:
//        /**
//         * @brief Z轴力控制（传统导纳）
//         */
//        static std::unique_ptr<AdmittanceController> createZAxis(
//            double m, double d, double k, double dt = 0.001)
//        {
//            IntegralParams params(m, d, k, dt);
//            return std::make_unique<AdmittanceController>(params, DOFMask::zOnly(), false);
//        }
//
//        /**
//         * @brief Z轴力控制（带积分）
//         */
//        static std::unique_ptr<AdmittanceController> createZAxisIntegral(
//            double m, double d, double k, double dt = 0.001,
//            double eta = 5.0, double integral_max = 50.0)
//        {
//            IntegralParams params(m, d, k, dt, eta, 1.0, integral_max, 0.1);
//            return std::make_unique<AdmittanceController>(params, DOFMask::zOnly(), true);
//        }
//
//        /**
//         * @brief 全6自由度（带积分）
//         */
//        static std::unique_ptr<AdmittanceController> create6DOF(
//            double m, double d, double k, double dt = 0.001, double eta = 5.0)
//        {
//            IntegralParams params(m, d, k, dt, eta);
//            return std::make_unique<AdmittanceController>(params, DOFMask::all(), true);
//        }
//
//        /**
//         * @brief XYZ平移控制（带积分）
//         */
//        static std::unique_ptr<AdmittanceController> createXYZ(
//            double m, double d, double k, double dt = 0.001, double eta = 5.0)
//        {
//            IntegralParams params(m, d, k, dt, eta);
//            return std::make_unique<AdmittanceController>(params, DOFMask::xyzOnly(), true);
//        }
//
//        /**
//         * @brief 预设场景
//         */
//        enum Preset { POLISHING, ASSEMBLY, GRINDING };
//
//        static std::unique_ptr<AdmittanceController> createPreset(
//            Preset preset, const DOFMask& mask = DOFMask::zOnly())
//        {
//            IntegralParams params;
//            params.dt = 0.001;
//
//            switch (preset) {
//            case POLISHING:  // 抛光：柔顺
//                params = IntegralParams(0.8, 12.0, 80.0, 0.001, 10.0);
//                break;
//            case ASSEMBLY:   // 装配：精确
//                params = IntegralParams(2.0, 25.0, 200.0, 0.001, 5.0);
//                break;
//            case GRINDING:   // 打磨：稳定
//                params = IntegralParams(1.5, 20.0, 150.0, 0.001, 8.0);
//                break;
//            }
//
//            return std::make_unique<AdmittanceController>(params, mask, true);
//        }
//    };
//
//} // namespace admittance
//
//#endif