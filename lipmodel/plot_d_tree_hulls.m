function plot_d_tree_hulls(bounds, a_min, a_max)
% bounds: two_step_bounds(...) 的返回结果
%         需包含 fields: dMin(7x1), dMax(7x1)
% a_min/a_max (可选): 参考加速度上/下界，画成虚线

% ------- 拆层（与你的 two_step_bounds 输出顺序一致） -------
% L1: d0_min, d0_max
L1 = [bounds.dMin(1) ; bounds.dMax(1)];

% L2: [d11_min, d11_max, d12_min, d12_max]
L2 = [bounds.dMin(2) ; bounds.dMax(2) ; ...
      bounds.dMin(3) ; bounds.dMax(3)];

% L3: [d21_min, d21_max, d22_min, d22_max, d23_min, d23_max, d24_min, d24_max]
L3 = [bounds.dMin(4) ; bounds.dMax(4) ; ...
      bounds.dMin(5) ; bounds.dMax(5) ; ...
      bounds.dMin(6) ; bounds.dMax(6) ; ...
      bounds.dMin(7) ; bounds.dMax(7)];

lev = {L1, L2, L3};                 % 每层节点
K   = numel(lev);                   % 层数 (=3)
t   = 1:K;                          % 层坐标：1,2,3

% ------- 每层凸包 [min,max] -------
H = zeros(K,2);
for k = 1:K
    H(k,1) = min(lev{k});
    H(k,2) = max(lev{k});
end

% ------- 画图 -------
figure; hold on; grid on; box on;

% (1) 半透明漏斗
xpoly = [t, fliplr(t)];
ypoly = [H(:,1).', fliplr(H(:,2).')];
fill(xpoly, ypoly, [0.2 0.6 1], 'FaceAlpha', 0.15, 'EdgeColor', 'none');

% (2) 上下边界
plot(t, H(:,1), 'b-', 'LineWidth', 1.5);
plot(t, H(:,2), 'b-', 'LineWidth', 1.5);

% (3) 节点散点
scatter(1*ones(numel(L1),1), L1, 40, 'k', 'filled');
scatter(2*ones(numel(L2),1), L2, 40, 'k', 'filled');
scatter(3*ones(numel(L3),1), L3, 40, 'k', 'filled');

% (4) 父—子连线（按二叉树顺序）
lc = [0.2 0.2 0.2]; lw = 1.1;

% L1(i) -> L2(2*i-1 : 2*i)
for i = 1:2
    ch = [2*i-1, 2*i];
    for c = ch
        plot([1,2], [L1(i), L2(c)], '-', 'Color', lc, 'LineWidth', lw);
    end
end

% L2(j) -> L3(2*j-1 : 2*j)
for j = 1:4
    ch = [2*j-1, 2*j];
    for c = ch
        plot([2,3], [L2(j), L3(c)], '-', 'Color', lc, 'LineWidth', lw);
    end
end

% (5) 参考加速度边界（可选）
if nargin >= 2 && ~isempty(a_min)
    yline(a_min, 'k--', 'LineWidth', 0.8);
end
if nargin >= 3 && ~isempty(a_max)
    yline(a_max, 'k--', 'LineWidth', 0.8);
end

xlabel('Tree level / step');
ylabel('Disturbance d');
title('Disturbance convex-hull propagation per level');
legend({'Hull area','lower/upper hull','Nodes'}, 'Location','best');
set(gca,'XTick',1:3,'XLim',[0.8 3.2]);
end
