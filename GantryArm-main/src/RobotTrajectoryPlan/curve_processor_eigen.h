#include <vector>
#include <map>
#include <array>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <numeric>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <sstream>
#include <iostream>
using namespace std;
using namespace Eigen;

struct Pose {
    Vector3d position;
    Quaterniond orientation;
};

class CurveProcessor
{
private:
    struct FaceData
    {
        vector<Vector3d> midpoints; // 该面的所有中点
        Vector3d normal;            // 面法线（统一）
    };
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit CurveProcessor(const std::string &csv_path);

    // 处理曲线对生成运动轨迹
    std::vector<Pose> generateMoveItTrajectory();

    std::vector<Eigen::Matrix4d> generateIKTrajectory();
    std::vector<Eigen::Matrix4d> generateOrientInpIKTrajectory(std::string file=" ");

    vector<pair<int, int>> pairs = { {7,8} ,{1,2},{3,4} };
    std::vector<std::pair<int, int>> side_pairs = { {9,10},{11,12} };

    // vector<pair<int, int>> pairs = {{1,2}};
    bool savePosesToCSV(const std::string& filename,const std::vector<Pose>& poses) ;
    map<int, FaceData> GetFaceDaa(){return _face_data;}

    void setRealCorners(const vector<Vector3d>& corners);
    void setRealCircles(const vector<Vector3d>& circles);

    // 生成侧面轨迹（Pose形式）
    std::vector<Pose> generateSideFaceTrajectory(double interval_mm = 1);
    std::vector<Pose> generateRowTrajectory(const std::vector<Eigen::Vector3d>& upper_points,
        const std::vector<Eigen::Vector3d>& lower_points,
        double sample_height,
        bool forward,
        const Eigen::Vector3d& face_normal);

    // 生成侧面轨迹的IK矩阵形式（带插补）
    std::vector<Eigen::Matrix4d> generateSideFaceIKTrajectory(std::string file = " ", double interval_mm = 20);

private:
    vector<Vector3d> _model_corners;  // 模型角点
    vector<Vector3d> _real_corners;   // 现实角点
    vector<Vector3d> _model_circles;  // 模型圆点
    vector<Vector3d> _real_circles;   // 现实圆点
    Matrix4d _transform_matrix;        // 变换矩阵
    map<int, vector<Vector3d>> _curve_data; // 修改数据结构为Vector3d容器
    Vector3d _centroid;                     // 工件质心
    map<int, FaceData> _face_data;          // 按面编号存储（key=0,1,2）

    Matrix4d computeRigidTransform(const vector<Vector3d>& modelPoints, 
                              const vector<Vector3d>& realPoints);
      
    void computeCentroid();    // 计算工件质心
    void validateCurvePairs(); // 校验曲线对是否合法
    void computeMidpoints();   // 计算中点轨迹
    // 加载角点数据的方法
    void loadCornerPoints();
    // 加载圆心点数据的方法
    void loadCirclePoints();
    // 变换所有曲线的方法
    void transformAllCurves();
    Vector3d computeTangent(
        const vector<Vector3d> &midpoints,
        size_t index,
        const Vector3d &face_normal // 接收当前面的法线！
    );
    Quaterniond  calculateOrientation(const Vector3d &tangent, const Vector3d &normal); // 姿态计算

    void computeFaceNormals(); // 新增：计算每个面的法线
    Vector3d computeNormalByThreePoints(const vector<Vector3d>& points) ;
    Vector3d computeNormalBySVD(const vector<Vector3d>& points) ;
    // 在 curve_processor_eigen.h 中添加声明
void debugRigidTransformBidirectional() {
    cout << "\n========== 刚体变换双向验证 ==========" << endl;
    
    if (_model_corners.size() != 8 || _real_corners.size() != 8) {
        cerr << "❌ 角点数量不对！" << endl;
        return;
    }
    
    // 计算变换矩阵的逆
    Matrix4d T_inv = _transform_matrix.inverse();
    
    cout << "\n【正向变换】Model → Real (使用 T)" << endl;
    cout << "变换矩阵 T:\n" << _transform_matrix << endl;
    cout << "旋转部分行列式: " << _transform_matrix.block<3,3>(0,0).determinant() << endl;
    
    double forward_total_error = 0.0;
    double forward_max_error = 0.0;
    
    for (size_t i = 0; i < 8; ++i) {
        // Model点变换到Real空间
        Vector4d model_homo;
        model_homo << _model_corners[i], 1.0;
        Vector3d transformed_real = (_transform_matrix * model_homo).head<3>();
        
        double error = (transformed_real - _real_corners[i]).norm();
        forward_total_error += error;
        forward_max_error = max(forward_max_error, error);
        
        cout << "点 " << i << ":" << endl;
        cout << "  Model原点:    " << _model_corners[i].transpose() << endl;
        cout << "  变换后(应=Real): " << transformed_real.transpose() << endl;
        cout << "  Real实际点:   " << _real_corners[i].transpose() << endl;
        cout << "  正向误差:     " << error * 1000 << " mm";
        if (error > 0.01) cout << " ⚠️";
        cout << endl;
    }
    
    cout << "\n正向统计:" << endl;
    cout << "  平均误差: " << forward_total_error / 8 * 1000 << " mm" << endl;
    cout << "  最大误差: " << forward_max_error * 1000 << " mm" << endl;
    
    cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << endl;
    
    cout << "\n【反向变换】Real → Model (使用 T^(-1))" << endl;
    cout << "逆变换矩阵 T^(-1):\n" << T_inv << endl;
    cout << "逆矩阵旋转部分行列式: " << T_inv.block<3,3>(0,0).determinant() << endl;
    
    double backward_total_error = 0.0;
    double backward_max_error = 0.0;
    
    for (size_t i = 0; i < 8; ++i) {
        // Real点逆变换到Model空间
        Vector4d real_homo;
        real_homo << _real_corners[i], 1.0;
        Vector3d transformed_model = (T_inv * real_homo).head<3>();
        
        double error = (transformed_model - _model_corners[i]).norm();
        backward_total_error += error;
        backward_max_error = max(backward_max_error, error);
        
        cout << "点 " << i << ":" << endl;
        cout << "  Real原点:     " << _real_corners[i].transpose() << endl;
        cout << "  逆变换后(应=Model): " << transformed_model.transpose() << endl;
        cout << "  Model实际点:  " << _model_corners[i].transpose() << endl;
        cout << "  反向误差:     " << error * 1000 << " mm";
        if (error > 0.01) cout << " ⚠️";
        cout << endl;
    }
    
    cout << "\n反向统计:" << endl;
    cout << "  平均误差: " << backward_total_error / 8 * 1000 << " mm" << endl;
    cout << "  最大误差: " << backward_max_error * 1000 << " mm" << endl;
    
    cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━" << endl;
    
    // 综合判断
    cout << "\n【综合判断】" << endl;
    bool forward_ok = forward_max_error < 0.01;  // <10mm
    bool backward_ok = backward_max_error < 0.01;
    
    if (forward_ok && backward_ok) {
        cout << "✅ 刚体变换矩阵正确！正反向误差都在合理范围内。" << endl;
    } else if (forward_ok && !backward_ok) {
        cout << "⚠️ 正向变换正常，但反向变换误差大！可能是数值精度问题。" << endl;
    } else if (!forward_ok && backward_ok) {
        cout << "⚠️ 反向变换正常，但正向变换误差大！检查_transform_matrix计算。" << endl;
    } else {
        cout << "❌ 正反向变换都有问题！" << endl;
        cout << "   可能原因：" << endl;
        cout << "   1. 角点对应顺序错误" << endl;
        cout << "   2. Model和Real点云不是刚体关系（有缩放/畸变）" << endl;
        cout << "   3. 输入数据单位不一致" << endl;
    }
    
    // 额外检查：T * T^(-1) 是否 = I
    Matrix4d should_be_identity = _transform_matrix * T_inv;
    Matrix4d diff = should_be_identity - Matrix4d::Identity();
    double identity_error = diff.norm();
    
    cout << "\n【数值稳定性检查】" << endl;
    cout << "T × T^(-1) 与单位矩阵的偏差: " << identity_error << endl;
    if (identity_error < 1e-10) {
        cout << "✅ 矩阵求逆精度良好" << endl;
    } else if (identity_error < 1e-6) {
        cout << "⚠️ 矩阵求逆精度一般（可能影响累积误差）" << endl;
    } else {
        cout << "❌ 矩阵求逆精度差！可能导致数值不稳定" << endl;
    }
    
    cout << "=======================================\n" << endl;
}

};
