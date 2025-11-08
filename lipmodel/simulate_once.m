function [ok, min_margin] = simulate_once(Amplitude, distbancePeriod, ...
    comHeight, stepDuration, averageSpeed, stepWidth, ...
    omega, Nsteps, deltaT, totalTime, fall_x_thresh)

% 重新初始化 footPlanner，避免跨仿真污染
footPlanner = scenairoTree_adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);

% 初值（与你代码一致）
currentStanceFootID       = 0;            % 0左脚支撑
currentStanceFootPosition = [0.0; 0.1];
xi = [0.03; 0];                           % DCM
stepDurationTic = round(stepDuration/deltaT);

% 正弦平台位移与二次导（加速度）
sinFunc = @(t) Amplitude * sin(2*pi/distbancePeriod * t);

ok = true;                 % 最终是否未摔倒
min_margin = inf;          % 记录全过程中最小的裕度(>0 越大越安全)

nSteps = round(totalTime/deltaT);
for k = 1:nSteps
    t = k*deltaT;

    % 平台位移x(t)与地面对机体的等效加速度扰动（只考虑x方向）
    % 注意：位移的二阶导 = -(2π/T)^2 * A * sin(2π/T * t)
    disturbance_x = -(2*pi/distbancePeriod)^2 * sinFunc(t);
    disturbance   = [disturbance_x; 0];

    % 调用你的脚步规划（带扰动），得到新的足端约束/落足
    footPlanner = footPlanner.findOptimalFootPlacement( ...
        Nsteps, xi, currentStanceFootID, currentStanceFootPosition, t, disturbance);

    % LIPM 更新（你的函数）
    xi = LIPModel(xi, omega, deltaT, currentStanceFootPosition, disturbance);

    % 更新支撑脚（以时间驱动的简单切换）
    currentStanceFootID = mod(floor((k+1)/stepDurationTic), 2);
    prevStanceFootID    = mod(floor(k/stepDurationTic), 2);
    if currentStanceFootID == prevStanceFootID
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(1); ...
                                  footPlanner.stanceFootConstraint.ankleY(1)];
    else
        nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(2); ...
                                  footPlanner.stanceFootConstraint.ankleY(2)];
    end
    currentStanceFootPosition = nextStanceFootPosition;

    % —— 摔倒判据（只看x方向）——
    dcm_offset_x = abs(xi(1) - currentStanceFootPosition(1));
    margin = fall_x_thresh - dcm_offset_x;   % >0 安全；<=0 触边界或越界
    if margin < min_margin
        min_margin = margin;
    end
    if dcm_offset_x > fall_x_thresh
        ok = false;   % 达到边界：记为不可行并退出
        return;
    end
end
end
