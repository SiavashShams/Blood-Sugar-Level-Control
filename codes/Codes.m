clc
disp("*********Q1*********")
A=[0,-4.5,0;0,-0.025,0.000013;0,0,-5.54/60];
B=[0;0;1/12];
C=[1 0 0];
D=0;
[b,a]=ss2tf(A,B,C,D);
Gs=tf(b,a)

%Q2
disp("*********Q2*********")
%pidTuner(Gs);
Kp = -22.73;
Ki = -0.0995;
Kd = -999.65;
C_pid = tf([Kd Kp Ki],[0 1 0]);
stepinfo(feedback(C_pid*Gs,1))
[y,t]=step(feedback(C_pid*Gs,1));
steady_state_error = abs(1-y(end))

%Q3
disp("*********Q3*********")
T1 = 1;
T2 = 2;
T3 = 5;
C_tustin_1 = c2d(C_pid,T1,'tustin')
C_tustin_2 = c2d(C_pid,T2,'tustin');
C_tustin_3 = c2d(C_pid,T3,'tustin');

C_matched_1 = c2d(C_pid,T1,'matched')
C_matched_2 = c2d(C_pid,T2,'matched');
C_matched_3 = c2d(C_pid,T3,'matched');
%stepinfo(out.simout.Data,out.simout.Time)
%stepinfo(out.simout1.Data,out.simout.Time)

%Q4
disp("*********Q4*********")
G_disc=c2d(Gs,T1)
figure(1)
rlocus(G_disc)
figure(11)
rlocus(-G_disc)
figure(2)
bode(G_disc)
figure(3)
margin(G_disc)
bw=bandwidth(G_disc)

%Q5
disp("*********Q5*********")
%Kp=-5.69;
%Ki=-0.00109;
%C_pid_disc=tf([Kp Kp-Ki],[1 -1],1)
% overshoot 10% and ts = 400s
zita = abs(log(0.1))/sqrt(pi^2+log(0.1)^2);
wn=4/zita/400;
z1 = exp(-zita*wn+i*wn*sqrt(1-zita^2))
z2 = exp(-zita*wn-i*wn*sqrt(1-zita^2))
theta = 180 - (atand(0.0135/(0.99-0.9753))+atand(0.0135/(0.99-0.9118)) - atand(0.0135/(0.99+0.2602)) - atand(0.0135/(0.99+3.6248))+180-atand(0.0135/(1-0.99)))
new_pole = 1-tand(theta)*0.0135
C_pid_disc_wo_k = tf([1 -0.9996],[1 -1],1)
K = 1/abs(evalfr(G_disc*C_pid_disc_wo_k,z2))
C_pid_disc = tf([-5.689, 5.687],[1 -1],1)

%Q6
disp("*********Q6*********")
figure(4)
rlocus(C_pid_disc*G_disc)
figure(5)
rlocus(feedback(C_pid_disc*G_disc,1))
figure(6)
margin(C_pid_disc*G_disc)
figure(7)
step(feedback(C_pid_disc*G_disc,1))
stepinfo(feedback(C_pid_disc*G_disc,1));
%Q7
disp("*********Q7*********")
z = tf('z')
ze = zero(G_disc)
f1=1/(1-ze(1));
f2=1-f1;
F=f1*z^-1+f2*z^-2
Gd=minreal(F/(1-F)/G_disc)
figure(8)
step(feedback(Gd*G_disc,1))

%Q8
disp("*********Q8*********")
% this part was done in simulink

%Q9
syms lambda
disp("*********Q9*********")
G = expm(A)
H = double(int(expm(A*lambda)*B,lambda,[0,1]))
C_new=C*G;
D_new=C*H;
% csys = canon(G_disc,'companion');
% A=csys.A'
% B=csys.C'
% C=csys.B'
% D=0
controllablity = ctrb(G,H)
observability = obsv(G,C_new)
controllablity = rank(ctrb(G,H))
disp("The system is controllable")
observability = rank(obsv(G,C_new))
disp("The system is observable")

%Q10
disp("*********Q10*********")
deadbeat = [0 0 1]*inv(ctrb(G,H))*(G^3)

%Q11
disp("*********Q11*********")
observer = ((G')^3)'*inv(ctrb(G,H))'*[0;0;1]
