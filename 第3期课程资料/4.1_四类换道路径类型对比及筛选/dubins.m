function path_dubins = dubins(A,F,d_temp)
d = 3.5;
B = [A(1) + d_temp,-d/2];
E = [F(1) - d_temp,d/2];
k = (E(2)-B(2)) / (E(1)-B(1));

% 求C点
syms x
y = @(x)[(x(1)-B(1))^2 + (x(2)-B(2))^2 - d_temp^2; (x(2)-B(2))/(x(1)-B(1)) - k];
C = fsolve(y,[B(1)+1,-1]);

% 求D点
syms x
y = @(x)[(x(1)-E(1))^2 + (x(2)-E(2))^2 - d_temp^2; (x(2)-E(2))/(x(1)-E(1)) - k];
D = fsolve(y,[E(1)-1,1]);

% 求第一个圆心
syms y
func = @(y)((y-C(2)) / (A(1)-C(1)) + 1/k);
y = fsolve(func,50);
center1 = [A(1),y];
R = norm(center1-A);

% 求第二个圆心
syms y
func = @(y)((y-D(2)) / (F(1)-D(1)) + 1/k);
y = fsolve(func,-50);
center2 = [F(1),y];

% 求theta角
OC = C - center1;
OA = A - center1;
theta = acos(OC * OA'/(norm(OC) * norm(OA)));

% 添加轨迹点
path = A;
for i = 0.001:0.001:theta
    path(end+1, 1:2) = [A(1) + R*sin(i), A(2) + R - R*cos(i)];
end
path = [path; C; D];
path_temp = [];
for i = 0.001:0.001:theta
    path_temp(end+1, 1:2) = [F(1) - R*sin(i), F(2) - R + R*cos(i)];
end

path_dubins = [path; path_temp(end:-1:1,:)];
