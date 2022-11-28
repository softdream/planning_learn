function B_spline_p=B_spline_func(P,n,k)
B_spline_p=[];
Nik = zeros(n+1, 1);

NodeVector = U_quasi_uniform(n, k); % 准均匀B样条的节点矢量
for u = 0 : 0.005 : 1-0.005
    for i = 0 : 1 : n
        Nik(i+1, 1) = BaseFunction(i, k , u, NodeVector);
    end
    p_u = P * Nik;
    B_spline_p=[B_spline_p;[p_u(1,1),p_u(2,1)]];
end

function Nik_u = BaseFunction(i, k , u, NodeVector)
% 计算基函数Ni,k(u),NodeVector为节点向量

if k == 0       % 0次B样条
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % 支撑区间的长度
    if Length1 == 0.0       % 规定0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end

function NodeVector = U_quasi_uniform(n, k)
% 准均匀B样条的节点向量计算，共n+1个控制顶点，k次B样条，k+1阶
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % 曲线的段数
if piecewise == 1       % 只有一段曲线时，n = k
    for i = k+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;       % 不止一段曲线时
    while flag ~= piecewise
        NodeVector(1, k+1+flag) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;
end

