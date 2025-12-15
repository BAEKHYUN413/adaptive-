clear all;
m_angle=0;
wrm_com=0;
wrm=0;
iqs=0;
iqs_reg=0;
iqs_com=0;
ids=0;
ids_reg=0;
ids_com=0;
dids=0;
diqs=0;
vqs=37;
vds=0;
vqs_unde =0;
vds_unde =0;

eq=0;
eq_reg=0;
ed=0;
ed_reg=0;
est_q=0;
est_d=0;
est_q_reg=0;
est_d_reg=0;
kq_int=0.5;
kd_int=0.5;
kq=kq_int;
kd=kd_int;

kiq = 440;
kid = 550;
int_eq = 0;
int_ed = 0;


Rs=3.65;
Ld=0.00783;
Lq=0.00979;
phif=0.063;
P=10;
ts=0.00002;
apha_q=0.11;
apha_d=0.07;
beta_q=0.03;
beta_d=0.005;
we=0;

WRM=[]; 
IQS=[]; 
IDS=[]; 
T=[];
IQS_COM=[]; 
IDS_COM=[]; 
EQ=[]; 
ED=[];
EST_D=[]; 
EST_Q=[]; 
Kq=[]; 
Kd=[];
VDS=[]; 
VQS=[];
IQS_COM_PREV=[]; 
IDS_COM_PREV=[];
iqs_com_prev = 0;
ids_com_prev = 0;
eq_prev = 0;
ed_prev = 0;
vqs_prev = 0;
vds_prev = 0;

for t=0:ts:1
    wrm=104.72;
    we=wrm*5;
    m_angle=m_angle+we*ts;

    iqs_com_prev = iqs_com;
    ids_com_prev = ids_com;

    if t >= 0.4 && t < 0.6
        ids_com = -0.216;
        iqs_com = 2.52;
    elseif t >= 0.0 && t < 0.4
        ids_com = -0.288;
        iqs_com = 2.988;
    elseif t >= 0.6
        ids_com = -0.288;
        iqs_com = 2.988;
    end

    %if iqs_com ~= iqs_com_prev
    %    kq = kq_int;
    %    int_eq = 0;  
    %end
    %if ids_com ~= ids_com_prev
    %    kd = kd_int;
    %    int_ed = 0;  
    %end

    vqs_prev = vqs;
    vds_prev = vds;
    eq_prev = eq;
    ed_prev = ed;
    eq = iqs_com - iqs;
    ed = ids_com - ids;

    %int_eq = int_eq + eq * ts;
    %int_ed = int_ed + ed * ts;

    %est_q = est_q + (1/apha_q) * eq / Lq * ts;
    %est_d = est_d + (1/apha_d) * ed / Ld * ts;

    %vqs = kq * eq + kiq * int_eq + est_q + we * Ld * ids + Rs * iqs + we * phif;
    %vds = kd * ed + kid * int_ed + est_d + Rs * ids - we * Lq * iqs;

    vqs = vqs_prev + kq * (eq - eq_prev) * ts + kiq * eq * ts ;
    vds = vds_prev + kd * (ed - ed_prev) * ts + kid * ed * ts ;

    %vqs = vqs_unde + we * ids * Ld + we * phif;
    %vds = vds_unde - we * iqs * Lq;
    


    diqs = -we * ((Ld+0.1*Ld)/(Lq+0.1*Lq)) * ids - (Rs+0.1*Rs)/(Lq+0.1*Lq) * iqs - (we * (phif + 0.1*phif))/(Lq + 0.1*Lq) + vqs/(Lq + 0.1*Lq);

    dids = -((Rs+0.1*Rs)/(Ld+0.1*Ld)) * ids + we * (Lq + 0.1*Lq)/(Ld + 0.1*Ld) * iqs + vds/(Ld + 0.1*Ld);

    iqs = iqs + diqs * ts;
    ids = ids + dids * ts;

    IQS_COM = [IQS_COM iqs_com];
    IDS_COM = [IDS_COM ids_com];
    IQS_COM_PREV = [IQS_COM_PREV iqs_com_prev];
    IDS_COM_PREV = [IDS_COM_PREV ids_com_prev];
    IQS = [IQS iqs];
    IDS = [IDS ids];
    EQ = [EQ eq];
    ED = [ED ed];
    EST_Q = [EST_Q est_q];
    EST_D = [EST_D est_d];
    Kq = [Kq kq];
    Kd = [Kd kd];
    VQS = [VQS vqs];
    VDS = [VDS vds];
    T = [T t];
end

%figure(1)
%plot(T, IQS, 'k', T, IQS_COM, 'k:')
%xlabel('time (sec)')
%ylabel('I_q_s (A)')
%legend('I_q_s^e','I^e_q_s^*')

%figure(2)
%plot(T, IDS, 'k', T, IDS_COM, 'k:')
%xlabel('time (sec)')
%ylabel('I_d_s (A)')
%legend('I_d_s^e','I^e_d_s^*')

figure(1)
subplot(2,1,1)
plot(T, IQS)
xlabel('time (sec)')
ylabel('i_q_s (A)')

subplot(2,1,2)
plot(T, IDS)
xlabel('time (sec)')
ylabel('i_d_s (A)')

figure(3)
subplot(2,1,1)
plot(T, VQS)
xlabel('time (sec)')
ylabel('v_q_s (V)')

subplot(2,1,2)
plot(T, VDS)
xlabel('time (sec)')
ylabel('v_d_s (V)')

figure(4)
plot(T, Kq)
xlabel('time (sec)')
ylabel('k_q')

figure(5)
plot(T, Kd)
xlabel('time (sec)')
ylabel('k_d')

%figure(6)
%plot(T, EST_Q)
%xlabel('time (sec)')
%ylabel('EST (Q)')

%figure(7)
%plot(T, EST_D)
%xlabel('time (sec)')
%ylabel('EST (D)')
