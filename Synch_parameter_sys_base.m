%Synch_parameter_sys_base
machine_numb = mac_con(:,1);
machine_base_mva=mac_con(:,2);
s_xls=mac_con(:,3).*system_base_mva./machine_base_mva;
%Armature leakage reactance
s_Rs=mac_con(:,4).*system_base_mva./machine_base_mva;
%Armature resistance
s_xd=mac_con(:,5).*system_base_mva./machine_base_mva;
%d axis synchronous reactance
s_xdd=mac_con(:,6).*system_base_mva./machine_base_mva;
%d axis transient reactance 
s_xddd=mac_con(:,7).*system_base_mva./machine_base_mva;
%d axis sub-transient reactance
s_Tdod=mac_con(:,8); %d axis open circuit transient time constant
s_Tdodd=mac_con(:,9); %d axis open circuit sub-transient time constant
s_xq=mac_con(:,10).*system_base_mva./machine_base_mva; %q axis synchronous reactance
s_xqd=mac_con(:,11).*system_base_mva./machine_base_mva; %q axis transient reactance
s_xqdd=mac_con(:,12).*system_base_mva./machine_base_mva; %q axis sub-transient reactance
s_Tqod=mac_con(:,13); %q axis open circuit transient time constant
s_Tqodd=mac_con(:,14); %q axis open circuit sub-transient time constant
s_H= mac_con(:,15).*system_base_mva./machine_base_mva; %Inertia constant
s_D=mac_con(:,16).*system_base_mva./machine_base_mva; %Self-damping
s_Rgov = s_Rgov.*system_base_mva./machine_base_mva;

ExpandSmachs= zeros(Nbus,NSmachs);
for counter=1:NSmachs
ExpandSmachs(Smachs(counter),counter)=1;
end

SelectSmachs= zeros(NSmachs,Nbus);
for counter=1:NSmachs
SelectSmachs(counter,Smachs(counter))=1;
end

s_Pg = bus_sol(Smachs,4);
%generator real power in machine base
s_Qg = bus_sol(Smachs,5);
%generator reactive power in machine base