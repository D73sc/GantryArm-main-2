% 打开文件以写入Markdown格式
fileID = fopen('D:\Git\GantryArm\GantryArmMatlab\InverseKinematics1.md', 'w');  % 创建并打开output.md文件

% 检查文件是否打开成功
if fileID == -1
    error('File open failed. Please check the file path and permissions.');
end

% 在Markdown文件中添加标题
fprintf(fileID, '# Homogeneous Transformation Matrices from Base to Joint\n\n');

% 确保使用符号变量
syms theta1 theta2 theta3 theta4 theta5 theta6 theta7 real  % Define all joint angles as symbolic variables

% Define MDH parameters as symbolic variables
a = sym([90, 200, 0, 0, 0, 0, 0]);   % Link lengths
alpha = sym([0, pi/2, pi/2, 0, -pi/2, 0, pi/2]);  % Twist angles
d = sym([-160-135.5, 0, 152.5, 228.5, 118, -118, 290]);  % Offsets
theta = [theta1, theta2 + pi/2, theta3, theta4, 0, 0, theta5];  % Joint angles

% Initialize cell to store homogeneous transformation matrices
T0 = eye(4);  % Store T0i (Transformation matrix from base to joint i)
T = cell(1, 7);  % Store Ti,i-1 (Transformation matrix from joint i-1 to joint i)

% Compute each homogeneous transformation matrix T0i and Ti,i-1, then output the formula
for i = 2:7
    % MDH formula to compute each transformation matrix Ti,i-1
    Ti_i_minus_1 = simplify([
        cos(theta(i)), -sin(theta(i)), 0, a(i);
        sin(theta(i)) * cos(alpha(i)), cos(theta(i)) * cos(alpha(i)), -sin(alpha(i)), -d(i) * sin(alpha(i));
        sin(theta(i)) * sin(alpha(i)), cos(theta(i)) * sin(alpha(i)), cos(alpha(i)), d(i) * cos(alpha(i));
        0, 0, 0, 1;
    ]);

    % Update the total transformation matrix T0i (from base to joint i)
    T0 = T0 * Ti_i_minus_1;
    
    % Output T0i (Transformation matrix from base to joint i) as LaTeX in Markdown
    fprintf(fileID, '## T_0%d\n\n', i);  % Title for each transformation matrix
    fprintf(fileID, '$$\n');  % Start of LaTeX block
    fprintf(fileID, '%s\n', latex(T0));  % Output the LaTeX formula of the matrix
    fprintf(fileID, '$$\n\n');  % End of LaTeX block

    % Output Ti,i-1 (Transformation matrix from joint i-1 to joint i) as LaTeX in Markdown
    fprintf(fileID, '## T_%d%d\n\n', i, i-1);  % Title for Ti,i-1 matrix
    fprintf(fileID, '$$\n');  % Start of LaTeX block
    fprintf(fileID, '%s\n', latex(Ti_i_minus_1));  % Output the LaTeX formula of Ti,i-1
    fprintf(fileID, '$$\n\n');  % End of LaTeX block
end

% Close the file
fclose(fileID);

disp('Markdown file has been saved as output.md');
