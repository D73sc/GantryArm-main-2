clc
clear
% 打开文件以写入Markdown格式
fileID = fopen('D:\Git\5DOFGantryArm\5DOFGantryArmMatlab\InverseKinematics8DOFFunction.md', 'w');  % 创建并打开output.md文件

% 检查文件是否打开成功
if fileID == -1
    error('File open failed. Please check the file path and permissions.');
end

% 在Markdown文件中添加标题
fprintf(fileID, '# Homogeneous Transformation Matrices from Base to Joint\n\n');

% 定义符号变量（论文式3-2）
syms nx ny nz ox oy oz ax ay az px py pz real   % 目标位姿矩阵参数(论文式3-2)
syms d0 d1 d2 theta3 theta4 theta5 theta6 theta7 Load real

% Define MDH parameters as symbolic variables
a = sym([0,0,0,90, 200, 0,0, 0]);   % Link lengths
alpha = sym([pi/2,pi/2,pi/2,0, pi/2, pi/2, pi/2,  pi/2]);  % Twist angles
d = sym([d0,d1,d2,160+135.5, 0, 152.5+228.5,0, 129+Load]);  % Offsets
theta = [pi/2,-pi/2,pi/2,theta3, theta4 + pi/2, theta5+ pi, theta6+ pi, theta7+ pi];  % Joint angles
% 目标位姿矩阵构造
T_goal = [nx ox ax px;      % 旋转矩阵R=[n o a]
          ny oy ay py;      % 位置向量p=[px;py;pz]
          nz oz az pz;
          0 0 0  1];
% Initialize cell to store homogeneous transformation matrices
T0 = eye(4);  % Store T0i (Transformation matrix from base to joint i)
T = cell(1, 8);  % Store Ti,i-1 (Transformation matrix from joint i-1 to joint i)
T7_8 = eye(4);  % Initialize as identity matrix
T0_7 = eye(4);  % Initialize as identity matrix
T7_8_inv = eye(4);  % Initialize as identity matrix

% Compute each homogeneous transformation matrix T0i and Ti,i-1, then output the formula
for i = 1:8
    % MDH formula to compute each transformation matrix Ti,i-1
    Ti_i_minus_1 = simplify([
        cos(theta(i)), -sin(theta(i)), 0, a(i);
        sin(theta(i)) * cos(alpha(i)), cos(theta(i)) * cos(alpha(i)), -sin(alpha(i)), -d(i) * sin(alpha(i));
        sin(theta(i)) * sin(alpha(i)), cos(theta(i)) * sin(alpha(i)), cos(alpha(i)), d(i) * cos(alpha(i));
        0, 0, 0, 1;
    ]);

    % Update the total transformation matrix T0i (from base to joint i)
    T0 = T0 * Ti_i_minus_1;
    
%     % Output T0i (Transformation matrix from base to joint i) as LaTeX in Markdown
%     fprintf(fileID, '## T_0%d\n\n', i);  % Title for each transformation matrix
%     fprintf(fileID, '$$\n');  % Start of LaTeX block
%     fprintf(fileID, '%s\n', latex(T0));  % Output the LaTeX formula of the matrix
%     fprintf(fileID, '$$\n\n');  % End of LaTeX block
% 
%     % Output Ti,i-1 (Transformation matrix from joint i-1 to joint i) as LaTeX in Markdown
%     fprintf(fileID, '## T_%d%d\n\n', i, i-1);  % Title for Ti,i-1 matrix
%     fprintf(fileID, '$$\n');  % Start of LaTeX block
%     fprintf(fileID, '%s\n', latex(Ti_i_minus_1));  % Output the LaTeX formula of Ti,i-1
%     fprintf(fileID, '$$\n\n');  % End of LaTeX block
    if i==7
       T0_7=T0; 
    end
    if i == 8
        T7_8 = Ti_i_minus_1;  % Save T6_8 transformation matrix
    end
    
end

% % Output T6_8 and T4_5 transformations as LaTeX in Markdown
% T0_3_inv=inv(T0_3);
% if ~isempty(T0_3_inv)
%     fprintf(fileID, '## T_0_3_inv\n\n');  % Title for T4_5
%     fprintf(fileID, '$$\n');  % Start of LaTeX block
%     fprintf(fileID, '%s\n', latex(T0_3_inv));  % Output the LaTeX formula of T4_5
%     fprintf(fileID, '$$\n\n');  % End of LaTeX block
% end
if ~isempty(T0_7)
    fprintf(fileID, '## T_0_7\n\n');  % Title for T4_5
    fprintf(fileID, '$$\n');  % Start of LaTeX block
    fprintf(fileID, '%s\n', latex(T0_7));  % Output the LaTeX formula of T4_5
    fprintf(fileID, '$$\n\n');  % End of LaTeX block
end
% if ~isempty(T5_8)
%     fprintf(fileID, '## T_5_8\n\n');  % Title for T6_8
%     fprintf(fileID, '$$\n');  % Start of LaTeX block
%     fprintf(fileID, '%s\n', latex(T5_8));  % Output the LaTeX formula of T6_8
%     fprintf(fileID, '$$\n\n');  % End of LaTeX block
% end
% T3_8=T3_5*T5_8;
% 
T7_8_inv=inv(T7_8);
if ~isempty(T7_8_inv)
    fprintf(fileID, '## T_7_8_inv\n\n');  % Title for T4_5
    fprintf(fileID, '$$\n');  % Start of LaTeX block
    fprintf(fileID, '%s\n', latex(T7_8_inv));  % Output the LaTeX formula of T4_5
    fprintf(fileID, '$$\n\n');  % End of LaTeX block
end
T_goal=T_goal*T7_8_inv;

if ~isempty(T_goal)
    fprintf(fileID, '## T_goal*T_7_8_inv\n\n');  % Title for T4_5
    fprintf(fileID, '$$\n');  % Start of LaTeX block
    fprintf(fileID, '%s\n', latex(T_goal));  % Output the LaTeX formula of T4_5
    fprintf(fileID, '$$\n\n');  % End of LaTeX block
end
% Close the file
fclose(fileID);

disp('Markdown file has been saved as output.md');
