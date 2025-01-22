function [Y] = form_Ymatrix(bs,ln);
% Form Y matrix from bus and line data

Y = zeros(size(bs,1),size(bs,1)); % Initialising Y bus to zero
sy = size(Y); % size of Y bus matrix

% This block of code adds line series admittance and shunt admittance to
% the diagonal elements of Y. First obtain index of diagonal elements
% corresponding to ln(:,1)
t = sub2ind(sy, ln(:,1), ln(:,1));
% Matrix Kt ensures correct admittance is added when more than one line
% connects two buses
[t1, t2, t3] = unique(t); Kt = zeros(size(t2,1),size(t3,1));
Kt(((1:size(Kt,2))-1)*size(Kt,1)+t3')=1;
% assigning values to diagonal elements
Y(t1) = Y(t1) + Kt*(1./(ln(:,3)+1i*ln(:,4))+1i*0.5*ln(:,5));

% This block does same as the previous block for the buses in ln(:,2)
t = sub2ind(sy, ln(:,2), ln(:,2));
[t1, t2, t3] = unique(t); Kt = zeros(size(t2,1),size(t3,1));
Kt(((1:size(Kt,2))-1)*size(Kt,1)+t3')=1;
Y(t1) = Y(t1) + Kt*(1./(ln(:,3)+1i*ln(:,4))+1i*0.5*ln(:,5));

% obtaining index of off-diagonal elements ln(:,1) to ln(:,2)
t = sub2ind(sy, ln(:,1), ln(:,2));
[t1, t2, t3] = unique(t); Kt = zeros(size(t2,1),size(t3,1));
Kt(((1:size(Kt,2))-1)*size(Kt,1)+t3')=1;
Y(t1) = Y(t1) - Kt*(1./(ln(:,3)+1i*ln(:,4)));

% obtaining index of off-diagonal elements ln(:,2) to ln(:,1)
t = sub2ind(sy, ln(:,2), ln(:,1));
[t1, t2, t3] = unique(t); Kt = zeros(size(t2,1),size(t3,1));
Kt(((1:size(Kt,2))-1)*size(Kt,1)+t3')=1;
Y(t1) = Y(t1) - Kt*(1./(ln(:,3)+1i*ln(:,4)));

% Adding shunt admittances at buses in the diagonal elements
Y = Y + diag(bs(:,8) + 1i*bs(:,9));

end