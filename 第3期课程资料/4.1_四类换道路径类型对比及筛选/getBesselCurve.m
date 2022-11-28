function path_bessel = getBesselCurve(P0, P1, P2, P3)
% 定义控制点
d = 3.5;
P = [P0; P1; P2; P3];

% 直接根据贝塞尔曲线定义式得到路径点
n = length(P)-1;
path_bessel = [];
for t = 0:0.01:1
    p_t = [0, 0];
    for i = 0:n
        k_C = factorial(n) / (factorial(i) * factorial(n-i));
        k_t = (1-t)^(n-i) * t^i;
        p_t = p_t + k_C * k_t * P(i+1,:);
    end
    path_bessel(end+1,:) = p_t;
end


