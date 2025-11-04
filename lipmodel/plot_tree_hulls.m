function plot_tree_hulls(b0, b_opt)
% b0    : 根节点(当前) DCM offset (scalar)
% b_opt : [bL2(1:2); bL3(1:4); bL4(1:8)] , size 14x1

assert(isvector(b_opt) && numel(b_opt)==14, 'b_opt must be 14x1');

% ----- 层索引 -----
idx.L2 = 1:2;          % level-2 (2 nodes)
idx.L3 = 3:6;          % level-3 (4 nodes)
idx.L4 = 7:14;         % level-4 (8 nodes)

% 每层节点集合
lev{1} = b0;                 % level-1 (root)
lev{2} = b_opt(idx.L2);      % level-2
lev{3} = b_opt(idx.L3);      % level-3
lev{4} = b_opt(idx.L4);      % level-4

% 每层凸包 [min,max]
H = zeros(4,2);
for k = 1:4
    H(k,1) = min(lev{k});
    H(k,2) = max(lev{k});
end
t = 0:3;

% ===== 画图 =====
figure; hold on; grid on; box on;

% 凸包漏斗
xpoly = [t, fliplr(t)];
ypoly = [H(:,1).', fliplr(H(:,2).')];
fill(xpoly, ypoly, [0.2 0.6 1], 'FaceAlpha', 0.15, 'EdgeColor','none');
plot(t, H(:,1), 'b-', 'LineWidth', 1.5);
plot(t, H(:,2), 'b-', 'LineWidth', 1.5);

% 节点散点
scatter(0, lev{1}, 40, 'k', 'filled');
scatter(1*ones(2,1), lev{2}, 40, 'k', 'filled');
scatter(2*ones(4,1), lev{3}, 40, 'k', 'filled');
scatter(3*ones(8,1), lev{4}, 40, 'k', 'filled');

% ===== 父—子连线 =====
lw = 1.1; lc = [0.2 0.2 0.2]; % 线宽/颜色

% root -> L2
for c = 1:2
    plot([0,1], [lev{1}, lev{2}(c)], '-', 'Color', lc, 'LineWidth', lw);
end

% L2 -> L3  （L3 的孩子索引 = [2*i-1, 2*i]）
for i = 1:2
    ch = [2*i-1, 2*i];             % 1->(1,2), 2->(3,4)
    for k = ch
        plot([1,2], [lev{2}(i), lev{3}(k)], '-', 'Color', lc, 'LineWidth', lw);
    end
end

% L3 -> L4  （L4 的孩子索引 = [2*j-1, 2*j]）
for j = 1:4
    ch = [2*j-1, 2*j];             % 1->(1,2), 2->(3,4), 3->(5,6), 4->(7,8)
    for k = ch
        plot([2,3], [lev{3}(j), lev{4}(k)], '-', 'Color', lc, 'LineWidth', lw);
    end
end

xlabel('Tree level / step');
ylabel('DCM offset b');
title('Convex-hull propagation per level (with parent-child edges)');
legend({'Hull area','lower/upper hull','Nodes'}, 'Location','best');
set(gca,'XTick',0:3,'XLim',[-0.2 3.2]);
end
