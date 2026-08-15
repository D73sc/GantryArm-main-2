classdef AdmittanceController
    % 导纳控制器类（支持传统导纳和带积分导纳）
    % 
    % 动力学方程：
    %   传统: M*? + C*? + K*x = F_ext - F_d
    %   积分: M*? + C*? + K*x = (F_ext - F_d) + η∫(F_ext - F_d)dt
    
    properties (Access = private)
        m       % 虚拟质量 [kg]
        c       % 虚拟阻尼 [Ns/m]
        k       % 虚拟刚度 [N/m]
        dt      % 采样时间 [s]
        
        % 积分参数
        use_integral    % 是否启用积分补偿
        eta             % 积分增益
        integral_max    % 积分限幅
        
        % 状态变量
        x       % 位置 [6x1]
        v       % 速度 [6x1]
        a       % 加速度 [6x1]
        integral  % 积分项 [6x1]
        
        F_d     % 期望力 [6x1]
    end
    
    methods
        function obj = AdmittanceController(m, c, k, dt, varargin)
            % 构造函数
            % 输入：
            %   m, c, k: 标量或6x1向量（质量、阻尼、刚度）
            %   dt: 采样时间
            %   可选: 'UseIntegral', true, 'Eta', 5.0
            
            % 参数解析
            p = inputParser;
            addRequired(p, 'm');
            addRequired(p, 'c');
            addRequired(p, 'k');
            addRequired(p, 'dt');
            addParameter(p, 'UseIntegral', false);
            addParameter(p, 'Eta', 5.0);
            addParameter(p, 'IntegralMax', 50.0);
            parse(p, m, c, k, dt, varargin{:});
            
            % 转换为6x1向量
            obj.m = obj.to6x1(m);
            obj.c = obj.to6x1(c);
            obj.k = obj.to6x1(k);
            obj.dt = dt;
            
            % 积分参数
            obj.use_integral = p.Results.UseIntegral;
            obj.eta = obj.to6x1(p.Results.Eta);
            obj.integral_max = obj.to6x1(p.Results.IntegralMax);
            
            % 初始化状态
            obj.x = zeros(6, 1);
            obj.v = zeros(6, 1);
            obj.a = zeros(6, 1);
            obj.integral = zeros(6, 1);
            obj.F_d = zeros(6, 1);
        end
        
        function pos = get_position(obj, F_ext)
            % 更新控制器并返回位置
            % 输入：F_ext [6x1] - 外部力/力矩
            % 输出：pos [6x1] - 位置/姿态
            
            if nargin < 2
                F_ext = zeros(6, 1);
            end
            
            % 力误差
            F_error = F_ext - obj.F_d;
            
            % 积分项更新
            if obj.use_integral
                obj.integral = obj.integral + F_error * obj.dt;
                % 限幅
                obj.integral = max(min(obj.integral, obj.integral_max), -obj.integral_max);
            end
            
            % 计算加速度
            if obj.use_integral
                F_total = F_error + obj.eta .* obj.integral;
            else
                F_total = F_error;
            end
            
            obj.a = (F_total - obj.c .* obj.v - obj.k .* obj.x) ./ obj.m;
            
            % 欧拉积分
            obj.v = obj.v + obj.a * obj.dt;
            obj.x = obj.x + obj.v * obj.dt;
            
            pos = obj.x;
        end
        
        function state = get_state(obj)
            % 返回完整状态 [x; v; a]
            state = [obj.x; obj.v; obj.a];
        end
        
        function acc = get_acceleration(obj)
            % 返回加速度
            acc = obj.a;
        end
        
        function vel = get_velocity(obj)
            % 返回速度
            vel = obj.v;
        end
        
        function int = get_integral(obj)
            % 返回积分项
            int = obj.integral;
        end
        
        function set_desired_force(obj, F_d)
            % 设置期望力
            obj.F_d = obj.to6x1(F_d);
        end
        
        function reset(obj)
            % 重置状态
            obj.x = zeros(6, 1);
            obj.v = zeros(6, 1);
            obj.a = zeros(6, 1);
            obj.integral = zeros(6, 1);
        end
    end
    
    methods (Static, Access = private)
        function vec = to6x1(val)
            % 将标量或向量转换为6x1
            if isscalar(val)
                vec = val * ones(6, 1);
            else
                vec = val(:);
                if length(vec) ~= 6
                    error('输入必须是标量或6维向量');
                end
            end
        end
    end
end