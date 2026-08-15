%% 带积分项的导纳控制器
classdef AdmittanceControllerWithIntegral
    properties
        M       % 虚拟质量
        B       % 阻尼
        K       % 刚度
        eta     % 积分增益分子
        k_e     % 积分增益分母
        dt      % 采样时间
        
        % 状态变量 [位置误差, 速度误差, 力误差积分]
        x       % x - x_d
        v       % ? - ?_d
        F_int   % ∫(F_d - F_e)dt
    end
    
    methods
        % 构造函数
        function obj = AdmittanceControllerWithIntegral(M, B, K, eta, k_e, dt)
            obj.M = M;
            obj.B = B;
            obj.K = K;
            obj.eta = eta;
            obj.k_e = k_e;
            obj.dt = dt;
            
            % 初始化状态
            obj.x = 0;
            obj.v = 0;
            obj.F_int = 0;
        end
        
        % 更新函数
        function [obj, x_actual] = update(obj, F_e, F_d, x_d, v_d, a_d)
            % 输入：
            %   F_e: 测量的外部力
            %   F_d: 期望力
            %   x_d: 期望位置
            %   v_d: 期望速度
            %   a_d: 期望加速度
            
            % 计算力误差积分的导数
            F_int_dot = F_d - F_e;
            
            % 计算加速度误差（从方程(4)推导）
            % M(? - ?_d) = F_e + (η/k_e)∫(F_d-F_e)dt - B(?-?_d) - K(x-x_d)
            a_error = (F_e + (obj.eta/obj.k_e)*obj.F_int - obj.B*obj.v - obj.K*obj.x) / obj.M;
            
            % 欧拉积分更新状态
            obj.x = obj.x + obj.v * obj.dt;
            obj.v = obj.v + a_error * obj.dt;
            obj.F_int = obj.F_int + F_int_dot * obj.dt;
            
            % 计算实际位置
            x_actual = obj.x + x_d;
        end
        
        % 重置状态
        function obj = reset(obj)
            obj.x = 0;
            obj.v = 0;
            obj.F_int = 0;
        end
    end
end
