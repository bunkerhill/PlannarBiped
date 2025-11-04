% 参数
Omega = omega;          % LIP 频率
A = -(2*pi/distbancePeriod)^2*Amplitude; ws = 2*pi/distbancePeriod;    % 正弦加速度参数
t0 = currentTime; Tc = 0.2; N = 3;


% 加速度函数句柄（正弦示例）
a_fun = @(tt) A*sin(ws*tt);


% 构造步端时刻
% t = t0 + (0:N)*Tc;
Tvec = footPlanner.stepDuration*ones(N,1);
Tvec(1) = footPlanner.leftoverTime;
t = zeros(N+1,1);
t(1) = t0;
for k = 1:N
    t(k+1) = t(k) + Tvec(k);
end
for k=1:N
    tk = t(k); tk1 = t(k+1);
    % 真值：用自适应数值积分
    d_true(k) = d_true_ode(a_fun, Omega, tk, tk1);
    % 闭式
    d_clos(k) = d_from_sine_closed(A, ws, Omega, tk, tk1);
end

abs_err = abs(d_clos - d_true);
rel_err = abs_err ./ max(1e-12, abs(d_true));

% 画图
figure; 
subplot(2,1,1); 
plot(1:N, d_true, 'o-', 1:N, d_clos, 'x-'); grid on;
xlabel('step k'); ylabel('d_k'); legend('numeric truth','closed form');
subplot(2,1,2); 
bar([abs_err, rel_err]); grid on;
xlabel('step k'); legend('abs error','rel error');

function dk = d_from_sine_closed(A, ws, Omega, tk, tk1)
    Dt = tk1 - tk;
    coef = A / (Omega*(Omega^2 + ws^2));
    term1 = Omega*sin(ws*tk1) + ws*cos(ws*tk1);
    term0 = Omega*sin(ws*tk ) + ws*cos(ws*tk );
    dk = coef * ( term1 - exp(Omega*Dt)*term0 );
end

function dk = d_true_numeric(a_handle, Omega, tk, tk1)
    kernel = @(tau) exp(Omega*(tk1 - tau)).*a_handle(tau);
    % 高精度设置，确保作为“真值”
    dk = -(1/Omega) * integral(kernel, tk, tk1, ...
               'RelTol',1e-10,'AbsTol',1e-12,'ArrayValued',true);
end

function dk = d_true_ode(a_handle, Omega, tk, tk1)
    ode = @(t,y) Omega*y + a_handle(t);
    opts = odeset('RelTol',1e-10,'AbsTol',1e-12);
    [~,Y] = ode45(ode, [tk tk1], 0, opts);
    y_end = Y(end);
    dk = -(1/Omega) * y_end;
end

