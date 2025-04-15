function F = fk_equations(pose, Q, CV, a, b, c)
    % fk_equations: 构造正向运动学约束方程组
    %
    % 输入：
    %   pose - 6×1 位姿向量 [x; y; z; roll; pitch; yaw]
    %   Q    - 8×1 绳长向量（已测量）
    %   CV   - 3×8 容器顶点坐标矩阵（每列为一个锚点的世界坐标）
    %   a, b, c - 平台尺寸（长、宽、高）
    %
    % 输出：
    %   F - 8×1 非线性约束向量，其中 F(i) = ||CV(:,i) - WAV(:,i)|| - Q(i)
    
    % 提取平台位姿
    x = pose(1); y = pose(2); z = pose(3);
    roll = pose(4); pitch = pose(5); yaw = pose(6);
    
    % 构造平台挂载点 MAV (3×8)
    hl = a / 2;
    hw = b / 2;
    hh = c / 2;
    MAV = [ hl,  hl,  hl,  hl, -hl, -hl, -hl, -hl;
            hw,  hw, -hw, -hw,  hw,  hw, -hw, -hw;
            hh, -hh,  hh, -hh,  hh, -hh,  hh, -hh];
    
    % 构造齐次变换矩阵 T
    % 注意：eul2rotm 这里采用 [yaw, pitch, roll] 顺序
    T = trvec2tform([x; y; z]) * eul2rotm([yaw, pitch, roll], 'ZYX');
    
    % 将平台挂载点 MAV 转换为齐次坐标形式（4×8）
    exMAV = [MAV; ones(1,8)];
    % 计算平台挂载点在世界坐标下的位置（4×8），取前三行
    exWAV = T * exMAV;
    WAV = exWAV(1:3,:);
    
    % 构造约束向量 F (8×1)，对于每个锚点：
    F = zeros(8, 1);
    for i = 1:8
        F(i) = norm(CV(:, i) - WAV(:, i)) - Q(i);
    end
end
