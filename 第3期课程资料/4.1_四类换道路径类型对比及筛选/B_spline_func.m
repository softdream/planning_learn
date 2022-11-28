function B_spline_p=B_spline_func(P,n,k)
B_spline_p=[];
Nik = zeros(n+1, 1);

NodeVector = U_quasi_uniform(n, k); % ׼����B�����Ľڵ�ʸ��
for u = 0 : 0.005 : 1-0.005
    for i = 0 : 1 : n
        Nik(i+1, 1) = BaseFunction(i, k , u, NodeVector);
    end
    p_u = P * Nik;
    B_spline_p=[B_spline_p;[p_u(1,1),p_u(2,1)]];
end

function Nik_u = BaseFunction(i, k , u, NodeVector)
% ���������Ni,k(u),NodeVectorΪ�ڵ�����

if k == 0       % 0��B����
    if (u >= NodeVector(i+1)) && (u < NodeVector(i+2))
        Nik_u = 1.0;
    else
        Nik_u = 0.0;
    end
else
    Length1 = NodeVector(i+k+1) - NodeVector(i+1);
    Length2 = NodeVector(i+k+2) - NodeVector(i+2);      % ֧������ĳ���
    if Length1 == 0.0       % �涨0/0 = 0
        Length1 = 1.0;
    end
    if Length2 == 0.0
        Length2 = 1.0;
    end
    Nik_u = (u - NodeVector(i+1)) / Length1 * BaseFunction(i, k-1, u, NodeVector) ...
        + (NodeVector(i+k+2) - u) / Length2 * BaseFunction(i+1, k-1, u, NodeVector);
end

function NodeVector = U_quasi_uniform(n, k)
% ׼����B�����Ľڵ��������㣬��n+1�����ƶ��㣬k��B������k+1��
NodeVector = zeros(1, n+k+2);
piecewise = n - k + 1;       % ���ߵĶ���
if piecewise == 1       % ֻ��һ������ʱ��n = k
    for i = k+2 : n+k+2
        NodeVector(1, i) = 1;
    end
else
    flag = 1;       % ��ֹһ������ʱ
    while flag ~= piecewise
        NodeVector(1, k+1+flag) = NodeVector(1, k + flag) + 1/piecewise;
        flag = flag + 1;
    end
    NodeVector(1, n+2 : n+k+2) = 1;
end

