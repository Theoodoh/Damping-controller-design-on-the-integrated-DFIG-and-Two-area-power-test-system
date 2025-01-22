function [bus_sln, flow] = power_flow(Y, bs, ln)
% The program solves load flow equations for a power system

bs(:,3) = bs(:,3)*pi/180; % converting angle from degrees to radians
V0 = bs(:,2); A0 = bs(:,3); nbs = size(bs,1);
bs1 = find(bs(:,10)==1); bpv = find(bs(:,10)==2); ...
    bpq = find(bs(:,10)==3);

% Active and reactive power specified at pv and pQ buses

PQ = [bs(bpv,4)-bs(bpv,6);bs(bpq,4)-bs(bpq,6); bs(bpq,5)-bs(bpq,7)];

% Initial estimate of voltage angle (pv and pQ bus) and magnitude (pQ bus)
vt0 = [bs(sort([bpv;bpq]),3);bs(bpq,2)];

itrn = 1;
while(true)
    % updating voltage magnitude and angle
    T = [zeros(size(bpq,1),size(bpq,1)+size(bpv,1))...
eye(size(bpq,1))];
    V0(bpq) = T*vt0;
    T = [eye(size(bpv,1)+size(bpq,1))...
        zeros(size(bpq,1)+size(bpv,1),size(bpq,1))];
    
    A0(sort([bpv; bpq])) = T*vt0;
 % Calculate voltage, current and power based on the estimate
    v0 = V0.*exp(1i*A0); i0 = Y*v0; pq0 = v0.*conj(i0);
    
 % Find difference in active and reactive power
    dpq = PQ-[real(pq0(sort([bpv;bpq])));imag(pq0(sort(bpq)))];
    
    K = diag(v0)*conj(Y)*diag(conj(v0)); % Calculating K matrix
    
    % Building Jacobian Matrix from K matrix
    Jp = [imag(K)-diag(imag(pq0)) ...
        (real(K)+diag(real(pq0)))./(ones(nbs,1)*V0')];
    
    Jq = [diag(real(pq0))-real(K) ...
        (imag(K)+diag(imag(pq0)))./(ones(nbs,1)*V0')];
    
    J = [Jp;Jq];
    J(sort([bs1; nbs+bs1;nbs+bpv]),:)=[];
    J(:,sort([bs1; nbs+bs1;nbs+bpv]))=[];
    
    dvt = J\dpq; % Finding change in voltage magnitude and angle
    
    vt0 = vt0 + dvt; % Updating voltage magnitude and angle
    
    itrn = itrn + 1;
    if max(abs(dvt))<1e-16 || itrn >50
        break
    end
end
% Building solved bus matrix
bus_sln = bs;
bus_sln(:,2:3) = [abs(v0) angle(v0)*180/pi];
bus_sln(sort([bpv;bs1]),4:5) = [real(pq0(sort([bpv;bs1])))...
    imag(pq0(sort([bpv;bs1])))];
% bus_sln(sort(bpq),6:7) = -[real(pq0(sort(bpq)))...
%     imag(pq0(sort(bpq)))]

% calculating line flow
sln = size(ln,1); flow = zeros(2*sln,4);
t = sub2ind(size(Y), ln(:,1), ln(:,2));
yft = Y(t);
flw =-v0(ln(:,1)).*conj(((v0(ln(:,1))-v0(ln(:,2))).*yft));
flow(1:sln, 1:4) = [ln(:,1) ln(:,2) real(flw) imag(flw)];

flw = -v0(ln(:,2)).*conj(((v0(ln(:,2))-v0(ln(:,1))).*yft));
flow(sln+1:2*sln, 1:4) = [ln(:,2) ln(:,1) real(flw) imag(flw)];
        