clear all
clc
close all
%% KUKA KR10 R1100-2
repo = fileparts(fileparts(mfilename('fullpath')));
a3 = 0.025;   % desfase vertical codo -> antebrazo [m]
d6 = 0.090;   % centro de muneca -> brida (tool0) [m]
% HOME definido en el SRDF [deg] (joint_1 ... joint_6)
q = deg2rad([0 -90 90 0 0 0]);

%% Direct Kinematic Model
% DH Matrix - DH=[a_ij alpha_ij s_j theta_j]   (unidades: metros)
DH=[   0      deg2rad(0)    0.400  -q(1);             % T_0^1  base -> A1
       0.025  deg2rad(-90)  0       q(2);             % T_1^2  A1 -> A2
       0.560  deg2rad(0)    0       q(3)-deg2rad(90); % T_2^3  A2 -> A3
       a3     deg2rad(-90)  0.515  -q(4);             % T_3^4  A3 -> A4
       0      deg2rad(90)   0       q(5);             % T_4^5  A4 -> A5
       0      deg2rad(-90)  d6      deg2rad(180)-q(6)]; % T_5^6 A5 -> tool0
frame=size(DH,1);

for i=1:frame
    T_ij(:,:,i)=T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4)); %MARCO LOCAL
end
T(:,:,1)=T_ij(:,:,1);
for i=1:frame-1
    T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1);
end
T_06=T(:,:,frame);
T_06(abs(T_06)<1e-12)=0;
disp('T_0^6 = T(base_link -> tool0) por DH:')
disp(T_06)

%% Comparacion con tf2_echo base_link tool0
tf_file=fullfile(repo,'resultados','tf_home.txt');
if isfile(tf_file)
    txt=fileread(tf_file);
    num=@(pat) str2double(strsplit(string(regexp(txt,pat,'tokens','once')),','));
    p_tf=num('Translation: \[([^\]]*)\]');
    q_tf=num('Quaternion \(xyzw\) \[([^\]]*)\]');
else
    p_tf=[0.630 0.000 0.985];        % Translation [x y z]
    q_tf=[0.000 0.707 0.000 0.707];  % Quaternion (xyzw)
end
qw=q_tf([4 1 2 3]); qw=qw/norm(qw);  % MATLAB usa [w x y z]
T_tf=[quat2rotm(qw) p_tf(:); 0 0 0 1];
disp('T(base_link -> tool0) de tf2_echo:')
disp(round(T_tf,4))
err_pos=norm(T_06(1:3,4)-T_tf(1:3,4));
err_ang=acos(max(-1,min(1,(trace(T_06(1:3,1:3)'*T_tf(1:3,1:3))-1)/2)));
fprintf('Error de posicion: %.3f mm | Error de orientacion: %.3f deg\n',1000*err_pos,rad2deg(err_ang))

%% Graph
% Define the Fixed Frame
hf=figure(1);
set(hf,'position',[445   275   563   628])
% Unit vectors scale factor(visual)
L=0.1;
h_ejexF=plot3([0 L],[0 0],[0 0],'r','linewidth',2.5);
hold on
h_ejeyF=plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
hold on
h_ejezF=plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
hold on
axis equal
axis([-0.3 1.3 -0.6 0.6 0 1.3])
grid on
xlabel('X [m]')
ylabel('Y [m]')
zlabel('Z [m]')
view(27,37)
% Mobile Coordinate System Based on DoF
for i=1:frame
    h_ejexM(i,1)=plot3([0 0],[0 0],[0 0],'r','linewidth',2);
    hold on
    h_ejeyM(i,1)=plot3([0 0],[0 0],[0 0],'g','linewidth',2);
    hold on
    h_ejezM(i,1)=plot3([0 0],[0 0],[0 0],'b','linewidth',2);
end
% Points Matrix on {j}
Mp=[0,0,0,1;
    L,0,0,1;
    0,L,0,1;
    0,0,L,1];
% Points Matrix on {0}
for k=1:frame
    for i=1:4
        Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')'; % MARCO GLOBAL
    end
end
% Plot each Coordinate System in the Global Frame
for k=1:frame
    set(h_ejexM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(2,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(2,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(2,3,k)])
    set(h_ejeyM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(3,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(3,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(3,3,k)])
    set(h_ejezM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(4,1,k)],...
        'ydata',[Mp_T(1,2,k) Mp_T(4,2,k)],...
        'zdata',[Mp_T(1,3,k) Mp_T(4,3,k)])
end
% Eslabones: une el origen de cada marco con el siguiente
O=[zeros(3,1) squeeze(Mp_T(1,1:3,:))];
plot3(O(1,:),O(2,:),O(3,:),'k-o','linewidth',1.5,'markerfacecolor','k')
title('KUKA KR10 R1100-2 en HOME')

%% Transform tipo Denavit-Hartenberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)
T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end
