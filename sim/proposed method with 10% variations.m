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

eq=0;
eq_reg=0;
ed=0;
ed_reg=0;
est_q=0;
est_d=0;
est_q_reg=0;
est_d_reg=0;
kq_int=3;
kd_int=5;
kq=kq_int;
kd=kd_int;

Rs=3.65;
Ld=0.00783;
Lq=0.00979;
phif=0.063;
P=10;
ts=0.00002;
apha_q=0.5;
apha_d=0.01;
beta_q=0.1; %0.0003%0.1  
beta_d=0.08;  %0.001%0.02
we=0;

WRM=[];
IQS=[];
IDS=[];
T=[];
IQS_COM=[];
IDS_COM=[];
ED=[];
EQ=[];
EST_D=[];
EST_Q=[];
Kq=[];
Kd=[];
V_3=[];
V_3_2=[];
VDS=[];
VQS=[];
iqs_com_prev = 0;
ids_com_prev = 0;


for t=0:ts:1
    wrm=104.72;
    we=wrm*5;
    m_angle=m_angle+we*ts;

    if t >= 0.5 && t < 0.6
        ids_com = -2;
        iqs_com = 3;
    else
        ids_com = -1;
        iqs_com = 2;
    end

    if iqs_com ~= iqs_com_prev
    kq = kq_int;   
    kq_reg = kq_int;
    eq = 0;        
    %est_q = 0;
    %est_q_reg = 0;
    iqs_com_prev = iqs_com;
    end


    IQS_COM=[IQS_COM iqs_com];
    eq_reg=eq;
    eq=iqs_com-iqs;
    est_q_reg=est_q;
    est_q=est_q_reg+((1/apha_q)*(eq/Lq)*ts);
    kq_reg=kq;
    kq=kq_reg+(1/beta_q)*eq^2/Lq*ts;
    Kq=[Kq kq];
    vqs=kq_reg*eq+est_q_reg+we*Ld*ids+Rs*iqs+we*phif;
    %diqs=-we*((Ld+Ld*0.1)/(Lq+Lq*0.1))*ids-(Rs+Rs*0.1)/(Lq+Lq*0.1)*iqs-(we*(phif+phif*0.1)/(Lq+Lq*0.1))+(vqs/(Lq+Lq*0.1));
    diqs=-(we*(Ld+Ld*0.1)/(Lq+Lq*0.1))*ids-((Rs+(Rs*0.1))/(Lq+Lq*0.1))*iqs-we*((phif+phif*0.1)/(Lq+Lq*0.1))+(1/(Lq+Lq*0.1))*vqs;
   
    if ids_com ~= ids_com_prev
    kd = kd_int;
    kd_reg = kd_int;
    ed = 0;
    %est_d = 0;
    %est_d_reg = 0;
    ids_com_prev = ids_com;
    end

    IQS=[IQS iqs];
    iqs=iqs+diqs*ts;
     
    IDS_COM=[IDS_COM ids_com];
    ed_reg=ed;
    ed=ids_com-ids;
    est_d_reg=est_d;
    est_d=est_d_reg+(1/apha_d)*ed/Ld*ts;
    kd_reg=kd;
    kd=kd_reg+(1/beta_d)*ed^2/Ld*ts;
    Kd=[Kd kd];
    vds=kd*ed+est_d+Rs*ids-we*Lq*iqs;
    %dids=-((Rs+Rs*0.1)/(Ld+Ld*0.1))*ids+(we*(Lq+Lq*0.1)/(Ld+Ld*0.1))*iqs+(vds/(Ld+Ld*0.1));
    dids=-((Rs+(Rs*0.1))/(Ld+Ld*0.1))*ids+(we*(Lq+Lq*0.1)/(Ld+Ld*0.1))*iqs+(1/(Ld+Ld*0.1))*vds;
  
    IDS=[IDS ids];
    ids=ids+dids*ts;

    ED=[ED ed];
    EQ=[EQ eq];
    
    EST_D=[EST_D est_d];
    EST_Q=[EST_Q est_q];

    VDS=[VDS vds];
    VQS=[VQS vqs];
    T=[T t];
    
end
figure(1)
plot(T, IQS, 'k', T, IQS_COM, 'k:')
xlabel('time (sec)')
ylabel('I_q_s (A)')
legend('I_q_s^e','I^e_q_s^*')

figure(2)
plot(T, IDS, 'k', T, IDS_COM, 'k:')
xlabel('time (sec)')
ylabel('I_d_s (A)')
legend('I_d_s^e','I^e_d_s^*')

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
%plot(T, EQ)
%xlabel('time (sec)')
%ylabel('I_q_s error (eq)')

%figure(7)
%plot(T, ED)
%xlabel('time (sec)')
%ylabel('I_q_s error (ed)')

figure(8)
plot(T, EST_Q)
xlabel('time (sec)')
ylabel('EST (Q)')

figure(9)
plot(T, EST_D)
xlabel('time (sec)')
ylabel('EST (D)')

