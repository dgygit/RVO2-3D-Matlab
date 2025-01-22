%--------------此版本使用ORCA，有宽松解-----------------%
%% 设定参数 set parameter
num = 2;            % 智能体数量 count of agents
glb = [-20000 -20000 -20000]; % 空间的下界 env limit
gub = [20000 20000 20000];    % 空间的上界 env limit
type = 'Circle';    % 初始化位置和目标的方法 method to initialize
maxv = 5000;           % 最大速度 max speed
bias = 0;           % 额外距离 extra dis
timehoriz = 100;     % 避障窗口 time horizon vo
acc = 100;            % 设定加速范围 accelerate limit
omega = 0.5;        % 设定转向范围 turn limit
ratio = 0.6;         % 设定避障比例 avoid ratio

%% 计算参数设置 parameters
dt = 0.1;          % 计算间隔 rate
maxduration = 10;   % 最长运行时间 max time

%% 初始化场景 initialize env
[pos, tar] = generate_initial_state(num, glb, gub, type);
vel = zeros(num,3); % 初始化速度为0 start speed
rds = 1000*ones(num,1);  % 初始化半径为1，也可以使用rand()随机化 radius
traj = cell(num,1);
for i=1:num
    traj{i}=[];
end

%% 绘图场景准备 initialize plot
figure;
[x, y, z] = sphere(10); 
balls = struct();
colors = parula(num);
for i=1:num
    balls(i).x = x.*rds(i);
    balls(i).y = y.*rds(i);
    balls(i).z = z.*rds(i);
    balls(i).color = colors(i,:);
end

%% 计算循环 main loop
t = 0:dt:maxduration;
for i=1:length(t)
    %% 计时 count time
    tic;
    newvel = zeros(num,3);
    %% calculate VO plane
    for k = 1:num
        prefv = FindPrefVel(pos(k,:), tar(k,:), maxv);
        [plane,maskstc] = OrcaNewPlane(pos(k,:),vel(k,:),[pos(1:k-1,:);pos(k+1:end,:)],[vel(1:k-1,:);vel(k+1:end,:)],...
            bias + rds(k) + [rds(1:k-1);rds(k+1:end)], timehoriz, dt, ratio);
        [lb,ub] = VelRange(vel(k,:), acc, omega, dt);
        newvel(k,:) = OrcaNewVel(plane,prefv,lb',ub',maskstc);
    end

    %% 更新速度位置 new vel
    vel = newvel;
    pos = pos + vel .* dt;
    
    %% 运行时间显示 time cost
    tt = toc;
    
    %% 绘图 plot
    tic;
    clf;
    axis equal;         % 设置轴
    axlim = [glb;gub];
    axis(axlim(:));
    hold on;
    view(-37.5,30);     % 设置视角
    grid on;
    
    for k=1:num
        surf(pos(k,1) + balls(k).x, pos(k,2) + balls(k).y, pos(k,3) + balls(k).z, 'FaceColor', balls(k).color,'EdgeColor', 'k');
    end
    st = toc;
    disp(['总消耗时间', num2str(st+tt), '秒']);
    %% 延时 delay to a specific rate
    pause(dt-tt-st);
    %% 记录 record
    for k=1:num
        traj{k}=[traj{k};pos(k,:)];
    end
end


figure;
axis equal;
hold on;
for i=1:num
    plot3(traj{i}(:,1), traj{i}(:,2), traj{i}(:,3), 'Color', balls(i).color);
end
