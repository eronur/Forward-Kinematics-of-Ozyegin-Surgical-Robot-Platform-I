% clc;clear variables;close all
%% State-Space Feedback Controller & State Observer Design for OSI by Onur Ersoy-15.03.2021
function [Af,Bf,Cf,Df] = StateFeedbackGain(fit,ts,p_discrete) %fit: the transfer function obtained from system identification bode plot fit, ts:sample time, p:nx1 vector that contains desired poles of the system in DISCRETE!!!
% load('C:\Users\onure\Dropbox\Control\ModelPredictiveControl\ContinuousTF3')
% load('C:\Users\student\Dropbox\surgery-EU\Control\ModelPredictiveControl\fit')
% load('C:\Users\onure\Dropbox\Control\ModelPredictiveControl\fit_s4s6')
% syms K1 K2 K3 K4
% ts=0.001;
%State-space transformation
Robot_SS1=ss(fit);
% Robot_SS2=ss(fit2_2);
% Robot_SS3=ss(fit2_3);
% Robot_SS4=ss(fit2_4);
% Robot_SS5=ss(fit2_5);

[A1,B1,C1,D1] = ssdata(Robot_SS1)
% [A2,B2,C2,D2] = ssdata(Robot_SS2);   
% [A3,B3,C3,D3] = ssdata(Robot_SS3);   
% [A4,B4,C4,D4] = ssdata(Robot_SS4);   
% [A5,B5,C5,D5] = ssdata(Robot_SS5);
%Observability and Controllability for continuous represtation
Ctrl_theta_s = ctrb(A1,B1) ;
if rank(Ctrl_theta_s) == min(size(Ctrl_theta_s))     
    disp('Controllable'); 
else
    disp('Uncontrollable');
end
Obs_theta_s = obsv(A1,C1);
if rank(Obs_theta_s) == min(size(Obs_theta_s))     
    disp('Observable'); 
else
    disp('Unobservable'); 
end

% Uncontrollable=[length(Robot_SS1.a)-rank(ctrb(Robot_SS1.a,Robot_SS1.b)),...
%                 length(Robot_SS2.a)-rank(ctrb(Robot_SS2.a,Robot_SS2.b)),...
%                 length(Robot_SS3.a)-rank(ctrb(Robot_SS3.a,Robot_SS3.b)),...
%                 length(Robot_SS4.a)-rank(ctrb(Robot_SS4.a,Robot_SS4.b)),...
%                 length(Robot_SS5.a)-rank(ctrb(Robot_SS5.a,Robot_SS5.b))];
% % Number of unobservable states
% Unobservable = [length(Robot_SS1.a)-rank(obsv(Robot_SS1.a,Robot_SS1.c)),...
%                 length(Robot_SS2.a)-rank(obsv(Robot_SS2.a,Robot_SS2.c)),...
%                 length(Robot_SS3.a)-rank(obsv(Robot_SS3.a,Robot_SS3.c)),...
%                 length(Robot_SS4.a)-rank(obsv(Robot_SS4.a,Robot_SS4.c)),...
%                 length(Robot_SS5.a)-rank(obsv(Robot_SS5.a,Robot_SS5.c))];
            
%Discrete state-space representation
dfit2_1=c2d(fit,ts,'tustin');%Conversion of continuous-time model to discrete time
[dfit2_1num,dfit2_1den]=tfdata(dfit2_1);
% dfit2_2=c2d(fit2_2,ts,'tustin');
% dfit2_3=c2d(fit2_3,ts,'tustin');
% dfit2_4=c2d(fit2_4,ts,'tustin');
% dfit2_5=c2d(fit2_5,ts,'tustin');

% [Ad1,Bd1,Cd1,Dd1] = ssdata(dfit2_1);   
[Ad1,Bd1,Cd1,Dd1] = c2dm(A1,B1,C1,D1,ts,'tustin');   
% [Ad2,Bd2,Cd2,Dd2] = ssdata(dfit2_2);   
% [Ad3,Bd3,Cd3,Dd3] = ssdata(dfit2_3);   
% [Ad4,Bd4,Cd4,Dd4] = ssdata(dfit2_4);   
% [Ad5,Bd5,Cd5,Dd5] = ssdata(dfit2_5);
%Observability and Controllability for discrete represtation
if rank(ctrb(Ad1,Bd1))==size(ctrb(Ad1,Bd1))     
    disp('SS - The Open Loop Discrete Position State Space form IS Controllable') 
else
    disp('SS - The Open Loop Discrete Position State Space form is NOT Controllable') 
end
if rank(obsv(Ad1,Cd1))==size(obsv(Ad1,Cd1))    
    disp('SS - The Open Loop Discrete Position State Space form IS Observable') 
else
    disp('SS - The Open Loop Discrete Position State Space form is NOT Observable')
end

%% STATE FEEDBACK CONTROLLER
%%Pole inspection for continuous time rep. (STATE FEEDBACK CONTROLLER)
% p=[-20 -20 -3 -3 -4 -4]; 
% % % 0.996+0.0605i 0.996-0.0605i zeros ofhe discrete ss
% K_=ackerC(A1,B1,p);
% sys=ss(A1-B1*K_,B1,C1,D1);
% G=-inv(C1*inv(A1-B1*K_)*B1);
% G=rscale(sys,K_)
% % % K_=[0.113 0.144 0.09 0.0893]
% % % [num,den]=tfdata(dfit2_2,'v');
% % % [R,P,K]=residue(num,den)


%%pole inspection for discrete time rep. (STATE FEEDBACK CONTROLLER)
% p_discrete = [0.9956 0.9956 0.4 0.4 0.9965+0.015i 0.9965-0.015i];%best result
% p_discrete = [0.9256 0.9256 0.9821 0.9821 0.45065 0.45065];%best result
% p_discrete = [0.992 0.992 0.997 0.997 0.99 0.99];%slower but more robust
% p_discrete = [0.922 0.922 0.9952 0.9952 0.29-0.15i 0.29+0.15i];%slower but more robust
% p_discrete = [0.95 0.95 0.95 0.95 0.96 0.96];%for tustin
% p_discrete = [0.8 0.8 0.7 0.7 0.7 0.7];%for foh
% % p_discrete = [-1 -1 -1 -1 -1 -1];% desired poles are selected according to pzmap of the transfer function
% % 
% Gain selection by Ackermann's formula
% Cd1 = eye(6);
% Dd1 = 0;
sysd=ss(Ad1,Bd1,Cd1,Dd1,ts);
Kd=ackerC(Ad1,Bd1,p_discrete);
% Gd=rscale(sysd,Kd);
% Gd=-inv(Cd1*inv(Ad1-Bd1*Kd)*Bd1);

%%STATE FEEDBACK CONTROLLER performance
sys2=ss(Ad1-Bd1*Kd,Bd1,Cd1,Dd1,ts);
% sys2=feedback(sysd,Kd)
N=1/(dcgain(sys2));
Jh=zeros(size(p_discrete))';
Jh(length(Jh)+1,1)=1;
N_=inv([Ad1-eye(6) Bd1; Cd1 0])*Jh;
N1=N_(1:6,1);
Nf=N_(7,1);
N=Nf+Kd*N1;
sys22=ss(Ad1-Bd1*Kd,Bd1*N,Cd1,Dd1,ts);

opt = stepDataOptions('StepAmplitude',1);
t=0:0.001:1;
P=timeoptions('cstprefs','SettleTimeThreshold');
step(sys22)
err=rms(step(sys22));
% figure(2)
% pzmap(sys22)
Eig=eig(Ad1-Bd1*Kd);
[z_sys22, g_sys22]=zero(sys22)
p_sys22=pole(sys22)
sys2_stable=isstable(sys22)
%% OBSERVER DESIGN
%%pole inspection
p_observer=[0.94 0.94 0.945 0.945 0.9425 0.9425];
% p_observer=[0.2 0.2 0.2 0.2 0.2 0.2];
Ko=acker(Ad1',Cd1',p_observer)';
Eig_o=eig(Ad1-Ko*Cd1);
%
dimA = length(Ad1); % A_theta_z is square so this is ok
Ao1 = [Ad1-Bd1*Kd -Bd1*Kd; zeros(dimA,dimA) Ad1-Ko*Cd1];
Bo1 = [Bd1*N; zeros(size(Bd1))];
Co1 = [Cd1 zeros(size(Cd1))];
Do1 = Dd1;
FullSys = ss(Ao1,Bo1,Co1,Do1,ts);

% step(FullSys)
% x0=[1 0 0 0 0 0];
% FullSys2 = ss(Ao1,Bo1/N,Co1,Do1,ts);
% [Y,T,X]=lsim(FullSys,zeros(size(t)),t,[x0 x0]);
% figure(4)
% step(FullSys2);
% plot([(X(:,(1:3))-X(:,(4:6)))])
% plot(X(:,(4:6)))
% grid on
% xlabel('Time (milliseconds)')
% ylabel('State Error')
% legend('e0','e1','e2')
% titlestring = ['SS - Controller/Observer States & Errors V.S. Time'];
% title(titlestring);
% x0=[0;0.1;0;0;0;0];
% x0_=[1;0;0;0;0;0];
% tic
% err=zeros(1,194481);

%% This loop calculates the poles that gives minimum step function output error for given State Feedback Gains. Long story short: Desired Poles
% n=0;
% uc=0.98:0.01:0.99;
% for i= 1:length(uc)
%     for j= 1:length(uc)
%         for k = 1:length(uc)
%             for h =1:length(uc)
%                 for f =1:length(uc)
%                     for v =1:length(uc)
%                         n=n+1;
%                         p_discrete=[uc(i) uc(j) uc(k) uc(h) uc(f) uc(v)]
%                         Kd=ackerC(Ad1,Bd1,p_discrete);
%                         sys2=ss(Ad1-Bd1*Kd,Bd1,Cd1,Dd1,0.001);
%                         N=1/(dcgain(sys2));
%                         if isnan(N)
%                             N=0;
%                             fprint('N is NA');
%                         else
%                             sys22=ss(Ad1-Bd1*Kd,Bd1*N,Cd1,Dd1,0.001);
%                         end
%                         err(n)=rms(step(sys22));
%                     end
%                 end
%             end
%         end
%     end
% end
% poleindex=find(err==min(err));
% toc
% err(poleindex(1))
% poleindex(1)
end
% clearvars -except A1 B1 C1 D1 Ad1 Bd1 Cd1 Dd1 dfit2_1 fit Kd Ko N Nf N1 ts
