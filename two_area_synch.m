system_base_mva = 100.0;

% Copy the code in the Script 3.8
% COPY BUS DATA FROM TABLE 2.5
% COPY LINE DATA FROM TABLE 2.6
%four_mac_data.m % bus No., voltage Mag., voltage Angle, pgen,
%Qgen, pload, Qload, pshunt, Qshunt, bus type
% bs = [...
% 1 1.00   00.0   7.000   0.00 0.0000   0.000   0.00  0.00 2;
% 2 1.00   00.0   7.000   0.00 0.0000   0.000   0.00  0.00 2;
% 3 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 1;
% 4 1.00   00.0   7.0000  0.00 0.0000   0.000   0.00  0.00 2;
% 5 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
% 6 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
% 7 1.00   00.0   0.0000  0.00 9.6700   1.000   0.00  2.00 3;
% 8 1.00   00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
% 9 1.00   00.0   0.0000  0.00 17.670   1.000   0.00  3.50 3;
% 10 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3;
% 11 1.00  00.0   0.0000  0.00 0.0000   0.000   0.00  0.00 3];
% 
% %from bus, To bus, Resistant, Inductance, Capacitance, tap-ratio, tap-phase
% ln = [01 05 0.0000 0.0167 0.00000 1.0 0.0;
%          02 06 0.0000 0.0167 0.00000 1.0 0.0;
%          03 11 0.0000 0.0167 0.00000 1.0 0.0;
%          04 10 0.0000 0.0167 0.00000 1.0 0.0;
%          05 06 0.0025 0.0250 0.04375 1.0 0.0;
%          10 11 0.0025 0.0250 0.04375 1.0 0.0;
%          06 07 0.0010 0.0100 0.01750 1.0 0.0;
%          09 10 0.0010 0.0100 0.01750 1.0 0.0;
%          07 08 0.0110 0.1100 0.19250 1.0 0.0;
%          07 08 0.0110 0.1100 0.19250 1.0 0.0;
%          08 09 0.0110 0.1100 0.19250 1.0 0.0;
%          08 09 0.0110 0.1100 0.19250 1.0 0.0];
% *********************** MACHINE DATA STARTS ***********************
% Machine data format
% 1. machine number,
% 2. bus number,
% 3. base mva,
% 4. leakage reactance x_l(pu),
% 5. resistance r_a(pu),
% 6. d-axis sychronous reactance x_d(pu),
% 7. d-axis transient reactance x'_d(pu),
% 8. d-axis subtransient reactance x"_d(pu),
% 9. d-axis open-circuit time constant T'_do(sec),
% 10. d-axis open-circuit subtransient time constant
% T"_do(sec),
% 11. q-axis sychronous reactance x_q(pu),
% 12. q-axis transient reactance x'_q(pu),
% 13. q-axis subtransient reactance x"_q(pu),
% 14. q-axis open-circuit time constant T'_qo(sec),
% 15. q-axis open circuit subtransient time constant
% T"_qo(sec),
% 16. inertia constant H(sec),
% 17. damping coefficient d_o(pu),
% 18. dampling coefficient d_1(pu),
% 19. bus number
% 20. saturation factor S(1.0) 
% 21. saturation factor S(1.2) 
% note: all the following machines use subtransient reactance model
mac_con =[
1 900 0.2 0.0025 1.8 0.3 0.25 8 0.03 1.7 0.55 0.25 0.4 0.05 6.5 0; 
2 900 0.2 0.0025 1.8 0.3 0.25 8 0.03 1.7 0.55 0.25 0.4 0.05 6.5 0;
3 900 0.2 0.0025 1.8 0.3 0.25 8 0.03 1.7 0.55 0.25 0.4 0.05 6.5 0;
4 900 0.2 0.0025 1.8 0.3 0.25 8 0.03 1.7 0.55 0.25 0.4 0.05 6.5 0];
% *********************** MACHINE DATA ENDS ***********************
% *********************** EXCITATION SYSTEM DATA ******************
s_Ka=200;%Static excitation gain Padiyar p.328
s_Ta=0.02;%Static excitation time constant Padiyar p.328
% **** Governor Control SYSTEM DATA – not used *********
s_Tg=0.2;%Kundur p.598
s_Rgov=0.05;%Kundur p.598
% Obtain load flow solution
[Y] = form_Ymatrix(bs,ln);
% calculate pre-fault power flow solution
[bus_sol, line_flow] = power_flow(Y,bs, ln);

% Synchronous machine initialisation

Smachs=[1;2;3;4];% buses with synchronous machines
Nbus=size(bs,1);% Number of buses
NSmachs=size(Smachs,1); %Number of synchronous machines

% program to initialise the synchronous machines
Synch_parameter_sys_base
generic_Sync_Init

% Copy the code in the Script 3.9
% Save this program as find_impedance_matrix.m
% Copy script from Script 2.5 here
% Calculate the vector of apparent power S injected by the generators for
% each system bus.
    PQ = bus_sol(:,4)+1i*bus_sol(:,5);
    
% Calculate the complex voltage value for all buses.
    vol = bus_sol(:,2).*exp(1i*bus_sol(:,3)*pi/180);
% Calculate the complex value of the current injected by the generators at
% each system bus
    Icalc = conj(PQ./vol);
    
% PL and QL are the real and reactive power drawn by loads at all system
% buses.

    PL = bus_sol(:,6);
    QL = bus_sol(:,7);
%V is the voltage magnitude at all system buses.

    V = bus_sol(:,2);
%The loads are assumed to be impedance type loads here and are included in
%the Y matrix. For more information on modelling loads, see chapter V. The
%Y matrix is then inverted to find the impedance matrix.
    YPL = PL./V.^2;
    YQL = QL./V.^2;
    Y = Y + diag(YPL-1i*YQL);
    Z = inv(Y);
% Finding post-fault impedance matrix
% Define post-fault admittance matrix
Yf = Y;
% Apply three phase fault at bus-8 by specifying very large admittance at
% Yf(8,8)
Yf(8,8) = 10000;
% Find post-fault impedance matrix
zf = inv(Yf);
