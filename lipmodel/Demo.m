%% ====== Common setup ======
clear; clc; close all;
addpath(genpath("./casadi-3"));

comHeight    = 0.525;           % m
stepDuration = 0.2;             % s
averageSpeed = 0.4;             % m/s
stepWidth    = 0.2;             % m
g = 9.8;  omega = sqrt(g/comHeight);
Nsteps   = 3;
deltaT   = 0.01;
totalTime= 5.0;                % 单次仿真时长
fall_x_thresh = 0.4;            % 摔倒判据

% 要求边界：对于每个 T，找到最大可行 Amplitude
T_list = linspace(0.5, 3.0, 5);  % 纵轴

%% ====== Binary search for A_max(T) ======
A_max = nan(size(T_list));
for iT = 1:numel(T_list)
    T = T_list(iT);

    % 搜索区间自己按需要放大/缩小
    lo = 0.0; 
    hi = 0.8;   % 假定幅值不会超过 0.8 m；如果不够就加大

    % 宽搜一次，确保 hi 是“不可行”（否则边界可能在区间右侧以外）
    ok_hi = simulate_once(hi, T, ...
        comHeight, stepDuration, averageSpeed, stepWidth, ...
        omega, Nsteps, deltaT, totalTime, fall_x_thresh);
    if ok_hi
        % 如果到 hi 都还可行，说明边界更大：记录为 hi 并提示扩大搜索区间
        A_max(iT) = hi;
        fprintf('[Warn] T=%.2f: boundary >= %.3f, consider increasing "hi".\n', T, hi);
        continue
    end

    % 二分搜索（12次约 1/4096 精度）
    for it = 1:12
        mid = 0.5*(lo + hi);
        ok  = simulate_once(mid, T, ...
            comHeight, stepDuration, averageSpeed, stepWidth, ...
            omega, Nsteps, deltaT, totalTime, fall_x_thresh);
        if ok
            lo = mid;   % 可行，往右扩
        else
            hi = mid;   % 不可行，往左收
        end
    end
    A_max(iT) = lo;      % lo 为最大可行幅值的近似
    fprintf('T=%.2f  ->  A_max=%.4f\n', T, A_max(iT));
end

%% ====== Plot boundary ======
figure; plot(A_max, T_list, 'k-', 'LineWidth', 2);
xlabel('Max Amplitude A_{max} (m)'); ylabel('Period T (s)');
title('Feasibility Boundary A_{max}(T)');
grid on;

%% ====== (Optional) also draw feasible mask around boundary for visualization ======
% 如果希望同时看“可行/不可行”分布，可在 A 方向再取若干采样点检查：
A_probe = linspace(0, max(A_max)*1.05, 60);
F = false(numel(T_list), numel(A_probe));
for iT = 1:numel(T_list)
    T = T_list(iT);
    for iA = 1:numel(A_probe)
        F(iT, iA) = simulate_once(A_probe(iA), T, ...
            comHeight, stepDuration, averageSpeed, stepWidth, ...
            omega, Nsteps, deltaT, totalTime, fall_x_thresh);
    end
end
figure;
imagesc(A_probe, T_list, F); set(gca,'YDir','normal');
xlabel('Amplitude (m)'); ylabel('Period T (s)');
title('Feasible Region (1=feasible, 0=fall)');
colorbar; colormap(parula);
hold on; plot(A_max, T_list, 'k-', 'LineWidth', 2);

%% ====== Subfunction: single run ======
function [ok, min_margin] = simulate_once(Amplitude, distbancePeriod, ...
    comHeight, stepDuration, averageSpeed, stepWidth, ...
    omega, Nsteps, deltaT, totalTime, fall_x_thresh)

    % 每次仿真都新建 planner，避免状态串扰
    footPlanner = scenairoTree_adaptiveFoot(comHeight, stepDuration, averageSpeed, stepWidth);

    currentStanceFootID       = 0;              % 0: left stance
    currentStanceFootPosition = [0.0; 0.1];
    xi = [0.03; 0];                              % DCM
    stepDurationTic = round(stepDuration/deltaT);

    sinFunc = @(t) Amplitude * sin(2*pi/distbancePeriod * t);

    ok = true; 
    min_margin = inf;

    nSteps = round(totalTime/deltaT);
    for k = 1:nSteps
        t = k*deltaT;

        % x方向平台位移二阶导 -> 加速度扰动
        disturbance_x = -(2*pi/distbancePeriod)^2 * sinFunc(t);
        disturbance   = [disturbance_x; 0];

        % 计划脚步（带扰动）
        footPlanner = footPlanner.findOptimalFootPlacement( ...
            Nsteps, xi, currentStanceFootID, currentStanceFootPosition, t, disturbance);

        % LIPM更新
        xi = LIPModel(xi, omega, deltaT, currentStanceFootPosition, disturbance);

        % 切换支撑脚
        currentStanceFootID = mod(floor((k+1)/stepDurationTic), 2);
        prevStanceFootID    = mod(floor(k/stepDurationTic),   2);
        if currentStanceFootID == prevStanceFootID
            nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(1); ...
                                      footPlanner.stanceFootConstraint.ankleY(1)];
        else
            nextStanceFootPosition = [footPlanner.stanceFootConstraint.ankleX(2); ...
                                      footPlanner.stanceFootConstraint.ankleY(2)];
        end
        currentStanceFootPosition = nextStanceFootPosition;

        % 摔倒判据 & 安全裕度
        dcm_offset_x = abs(xi(1) - currentStanceFootPosition(1));
        margin = fall_x_thresh - dcm_offset_x;
        if margin < min_margin, min_margin = margin; end
        if dcm_offset_x > fall_x_thresh
            ok = false;
            return;
        end
    end
end
