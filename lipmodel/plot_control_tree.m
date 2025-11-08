function plot_control_tree(s_opt, u_min, u_max)
% plot_control_tree  可视化 scenario-tree 控制输入的层级凸包与父子连线
% 输入:
%   s_opt : [u1; u2(1:2); u3(1:4)]  (7x1)
%   u_min,u_max (可选): 输入上下界, 画成水平虚线
%
% 说明:
%   level-1: u1 (共享)
%   level-2: u2(1:2)    —— 作为 u1 的两个子控制
%   level-3: u3(1:4)    —— u2(i) 的两个子控制 (索引 2*i-1 : 2*i)

    arguments
        s_opt (:,1) double
        u_min (1,1) double = NaN
        u_max (1,1) double = NaN
    end
    assert(numel(s_opt)==7, 's_opt must be 7x1: [u1; u2(2); u3(4)].');

    % ----- 分层 -----
    U1 = s_opt(1);
    U2 = s_opt(2:3);
    U3 = s_opt(4:7);

    lev = {U1, U2, U3};      % 每层节点
    K   = numel(lev);        % =3
    t   = 1:K;               % 层坐标：1,2,3

    % ----- 每层凸包 [min,max] -----
    H = zeros(K,2);
    for k = 1:K
        H(k,1) = min(lev{k});
        H(k,2) = max(lev{k});
    end

    % ===== 画图 =====
    figure; hold on; grid on; box on;

    % (1) 半透明漏斗
    xpoly = [t, fliplr(t)];
    ypoly = [H(:,1).', fliplr(H(:,2).')];
    fill(xpoly, ypoly, [0.2 0.6 1], 'FaceAlpha', 0.15, 'EdgeColor','none');

    % (2) 上下边界
    plot(t, H(:,1), 'b-', 'LineWidth', 1.5);
    plot(t, H(:,2), 'b-', 'LineWidth', 1.5);

    % (3) 节点散点
    scatter(1*ones(numel(U1),1), U1, 40, 'k', 'filled');   % 只有一个点
    scatter(2*ones(numel(U2),1), U2, 40, 'k', 'filled');
    scatter(3*ones(numel(U3),1), U3, 40, 'k', 'filled');

    % (4) 父—子连线
    lc = [0.2 0.2 0.2]; lw = 1.1;

    % u1 -> u2(1:2)
    for c = 1:2
        plot([1,2], [U1, U2(c)], '-', 'Color', lc, 'LineWidth', lw);
    end

    % u2(i) -> u3(2*i-1 : 2*i)
    for i = 1:2
        ch = [2*i-1, 2*i];      % 1->(1,2), 2->(3,4)
        for k = ch
            plot([2,3], [U2(i), U3(k)], '-', 'Color', lc, 'LineWidth', lw);
        end
    end

    % (5) 可选: 输入上下界
    if ~isnan(u_min), yline(u_min, 'k--', 'LineWidth', 0.8); end
    if ~isnan(u_max), yline(u_max, 'k--', 'LineWidth', 0.8); end

    xlabel('Tree level / step');
    ylabel('Control input s (step length/width)');
    title('Control inputs over the scenario tree (hulls + parent-child edges)');
    legend({'Hull area','lower/upper hull','Nodes'}, 'Location','best');
    set(gca,'XTick',1:3,'XLim',[0.8 3.2]);
end
