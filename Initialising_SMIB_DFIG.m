% Bus No., voltage, Angle, pq, Qg,pl, Ql, Gl, Bl, Bus type
bs = [...
1 1.00   00.0   7.000   0.00 0.0000   0.000   0.00  0.00 2;
2 1.00   00.0   7.000   0.00 0.0000   0.000   0.00  0.00 2;
3 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 1;
4 1.00   00.0   7.0000  0.00 0.0000   0.000   0.00  0.00 2;
5 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
6 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
7 1.00   00.0   0.0000  0.00 9.6700   1.000   0.00  2.00 3;
8 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
9 1.00   00.0   0.0000  0.00 17.670   1.000   0.00  3.50 3;
10 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
11 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
12 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
13 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
14 1.00  00.0   0.2000  0.0125 0.0000   0.000   0.00  0.00 3];

%from bus, To bus, Resistant, Inductance, Capacitance, tap-ratio, tap-phase
ln = [01 05 0.0000 0.0167 0.00000 1.0 0.0;
         02 06 0.0000 0.0167 0.00000 1.0 0.0;
         03 11 0.0000 0.0167 0.00000 1.0 0.0;
         04 10 0.0000 0.0167 0.00000 1.0 0.0;
         05 06 0.0025 0.0250 0.04375 1.0 0.0;
         10 11 0.0025 0.0250 0.04375 1.0 0.0;
         06 07 0.0010 0.0100 0.01750 1.0 0.0;
         09 10 0.0010 0.0100 0.01750 1.0 0.0;
         07 08 0.0110 0.1100 0.19250 1.0 0.0;
         07 08 0.0110 0.1100 0.19250 1.0 0.0;
         08 09 0.0110 0.1100 0.19250 1.0 0.0;
         08 09 0.0110 0.1100 0.19250 1.0 0.0;
         07 12 0.0000 0.0500 0.00000 1.0 0.0;
         12 13 0.0075 0.0125 0.02000 1.0 0.0;
         13 14 0.0000 0.0250 0.00000 1.0 0.0];
     
% obtain load flow solution
Y = form_Ymatrix(bs,ln);
% calculate pre-fault power flow solution
[bus_sln, line_flow] = power_flow(Y,bs, ln);
Znet = inv(Y);
vinf = bus_sln(3,2)*exp(1i*bus_sln(3,3)*pi/180);
ZA = Znet(1:end-1,1:end-1); ZB = Znet(1:end-1,end);
ZC = Znet(end,1:end-1); ZD = Znet(end,end); % Znet = [Za Zb; Zc Zd]
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Smachs = [1;2;3;4];  % Bus where SMIB is connected
Dmachs = [13];  % Bus where DFIG is connected
Omega = 2*pi*50;
if size(Smachs,1)
    
    two_area_synch
    MMST_mult = zeros(size(bus_sln,1)-1,size(Smachs,1));
    MMST_mult(Smachs,:) = eye(size(Smachs,1));
end

if size(Dmachs,1)
    
    find_dfig_state_initial_conditions
    dfig_mult = zeros(size(bus_sln,1)-1,size(Dmachs,1));
    dfig_mult(Dmachs,:) = eye(size(Dmachs,1));
end
d_vw = 14.5316; d_Lm = 4;        d_Rs = 0.005;     d_Rr = 0.0055;
d_Lss = 4.04;    d_Lrr = 4.0602;   d_Kopt =1;       d_ktg = 0.3;
d_ctg = 0.01;    d_Ht = 4;         d_Hg = 0.4;
d_ws =1; d_Tg=d_Ts;  d_wb = 2*pi*50;
d_w = 2*pi*50; d_vcq  = 0.9837;
d_MSC_IL1_kp = -0.23;  d_MSC_IL1_ki = -3;  d_MSC_IL1_iv = 0.0389;
d_MSC_IL2_kp = -0.23;  
d_MSC_OL1_kp = 0;      
d_MSC_OL2_kp = 0;      

d_GSC_IL1_kp = 0.3;  
d_GSC_IL2_kp = 0.3;  
d_GSC_OL1_kp = -22;  
d_GSC_OL2_kp = 0;

SelectSmachs = [1];
ExpandSmachs = [1];
s_Efd_max = 7; s_Efd_min = 0;

small_d=[
    0  1 1.1 10
    1  1.00 1 1;];
