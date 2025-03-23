clc;
clear;

% 初始化容器和长方体的尺寸
container_size = 45; % 正方体容器边长
length = 8; % 长方体长度
width = 10;  % 长方体宽度
height = 6; % 长方体高度

% 创建长方体的顶点
vertices = [
    -length/2 -width/2 -height/2;   % 1
    length/2 -width/2 -height/2;   % 2
    length/2 width/2 -height/2;   % 3
    -length/2 width/2 -height/2;   % 4
    -length/2 -width/2 height/2;   % 5
    length/2 -width/2 height/2;   % 6
    length/2 width/2 height/2;   % 7
    -length/2 width/2 height/2   % 8
];

% 长方体的面索引
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

% 创建图形
figure;
axis equal;
xlim([-container_size/2 container_size/2]);
ylim([-container_size/2 container_size/2]);
zlim([-container_size/2 container_size/2]);
grid on;
view(3);

% 绘制正方体容器的顶点和面
container_vertices = [
    -container_size/2, -container_size/2, -container_size/2;
    container_size/2, -container_size/2, -container_size/2;
    container_size/2, container_size/2, -container_size/2;
    -container_size/2, container_size/2, -container_size/2;
    -container_size/2, -container_size/2, container_size/2;
    container_size/2, -container_size/2, container_size/2;
    container_size/2, container_size/2, container_size/2;
    -container_size/2, container_size/2, container_size/2;
];

container_faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

% 绘制容器
hold on;
patch('Faces', container_faces, 'Vertices', container_vertices, 'FaceColor', 'g', 'FaceAlpha', 0.1);

% 初始化长方体的位置和旋转角度
position = [0, 0, 0]; % 初始位置
angles = [0, 0, 0]; % 初始旋转角度，绕x, y, z轴

% 设置旋转矩阵
rotation_matrix = @(angle, axis) ...
    [cos(angle) + axis(1)^2 * (1 - cos(angle)), axis(1)*axis(2)*(1 - cos(angle)) - axis(3)*sin(angle), axis(1)*axis(3)*(1 - cos(angle)) + axis(2)*sin(angle);
     axis(2)*axis(1)*(1 - cos(angle)) + axis(3)*sin(angle), cos(angle) + axis(2)^2 * (1 - cos(angle)), axis(2)*axis(3)*(1 - cos(angle)) - axis(1)*sin(angle);
     axis(3)*axis(1)*(1 - cos(angle)) - axis(2)*sin(angle), axis(3)*axis(2)*(1 - cos(angle)) + axis(1)*sin(angle), cos(angle) + axis(3)^2 * (1 - cos(angle))];
 
rotated_vertices = vertices;

% 初始化长方体的patch对象
h = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'b');

% 初始化长方体到容器顶点的红色连接线
h_line = gobjects(1, 8);  % 预先创建8个对象来存储每条红线
for i = 1:8
    h_line(i) = plot3([container_vertices(i,1), rotated_vertices(i,1)], ...
                      [container_vertices(i,2), rotated_vertices(i,2)], ...
                      [container_vertices(i,3), rotated_vertices(i,3)], 'Color', 'r', 'LineWidth', 2);
end

% 设置动画的时间步长
num_frames = 1000;
for k = 1:num_frames
    if k < 200
        position = [0, 0, 0];
        angles = [sin((k-100)*0.01)*pi/3, 0, 0];
    elseif k > 200 && k < 400
        position = [0, 0, 0];
        angles = [0, sin((k-300)*0.01)*pi/3, 0];
    elseif k > 400 && k < 600
        position = [0, 0, 0];
        angles = [0, 0, sin((k-500)*0.01)*pi/3];
    elseif k > 600 && k < 800
        position = [sin(k*0.05) * container_size / 5, cos(k*0.05) * container_size / 5, 0];
        angles = [0, 0, 0];
    elseif k > 800 && k < 1000
        position = [0, 0, sin(k*0.05) * container_size / 10];
        angles = [0, 0, 0];
    end

    % 计算旋转矩阵
    R_x = rotation_matrix(angles(1), [1, 0, 0]);
    R_y = rotation_matrix(angles(2), [0, 1, 0]);
    R_z = rotation_matrix(angles(3), [0, 0, 1]);
    R = R_x * R_y * R_z;
    
    % 旋转后的顶点
    rotated_vertices = (R * vertices')';
    rotated_vertices = rotated_vertices + position;
    
    % 更新长方体的顶点
    h.Vertices = rotated_vertices;
    
    % 更新连接线
    for i = 1:8
        set(h_line(i), 'XData', [container_vertices(i,1), rotated_vertices(i,1)], ...
                        'YData', [container_vertices(i,2), rotated_vertices(i,2)], ...
                        'ZData', [container_vertices(i,3), rotated_vertices(i,3)]);
    end
    
    % 强制刷新图形
    drawnow;
end
