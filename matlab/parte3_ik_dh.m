clear all
clc
close all
%% KUKA KR10 R1100-2 - Parte 3: IK de MoveIt2 (KDL) vs modelo DH
% 1) ros2 run kr10_taller ik_poses  -> pide a MoveIt la IK y guarda resultados/ik_resultados.json
% 2) Aqui esos angulos entran al modelo DH (modificado / Craig, igual que Simulacion_dh.m)
% 3) Se compara T_DH contra la pose objetivo y contra la FK de MoveIt
repo = fileparts(fileparts(mfilename('fullpath')));
a3 = 0.025; d6 = 0.090;

data = jsondecode(fileread(fullfile(repo,'resultados','ik_resultados.json')));
nombres = fieldnames(data.poses);

hf=figure(1);
set(hf,'position',[200 200 1000 520])
for n=1:numel(nombres)
    P = data.poses.(nombres{n});
    q = P.q_rad(:)';

    %% Direct Kinematic Model
    % DH Matrix - DH=[a_ij alpha_ij s_j theta_j]
    DH=[   0      deg2rad(0)    0.400  -q(1);
           0.025  deg2rad(-90)  0       q(2);
           0.560  deg2rad(0)    0       q(3)-deg2rad(90);
           a3     deg2rad(-90)  0.515  -q(4);
           0      deg2rad(90)   0       q(5);
           0      deg2rad(-90)  d6      deg2rad(180)-q(6)];
    frame=size(DH,1);
    clear T_ij T
    for i=1:frame
        T_ij(:,:,i)=T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4)); %MARCO LOCAL
    end
    T(:,:,1)=T_ij(:,:,1);
    for i=1:frame-1
        T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1);
    end
    T_06=T(:,:,frame);

    %% Comparacion
    T_obj = pose2T(P.target_p, P.target_q_xyzw);   % pose pedida a MoveIt
    T_fk  = pose2T(P.fk_p, P.fk_q_xyzw);           % pose que MoveIt calcula con la IK
    fprintf('================ %s ================\n', upper(nombres{n}));
    fprintf('q IK MoveIt [deg] = [%s]\n', num2str(rad2deg(q),'%9.3f'));
    disp('T_0^6 por DH:');            disp(round(T_06,4));
    disp('T objetivo (MoveIt):');     disp(round(T_obj,4));
    [ep,ea]=errores(T_06,T_obj);
    fprintf('DH vs objetivo : %.4f mm | %.4f deg\n', ep, ea);
    [ep,ea]=errores(T_06,T_fk);
    fprintf('DH vs FK MoveIt: %.4f mm | %.4f deg\n\n', ep, ea);

    %% Graph (mismo esquema de marcos que Simulacion_dh.m)
    subplot(1,2,n)
    L=0.1;
    plot3([0 L],[0 0],[0 0],'r','linewidth',2.5); hold on
    plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
    plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
    Mp=[0,0,0,1; L,0,0,1; 0,L,0,1; 0,0,L,1];   % Points Matrix on {j}
    O=zeros(3,frame+1);
    for k=1:frame
        for i=1:4
            Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')'; % MARCO GLOBAL
        end
        plot3(Mp_T([1 2],1,k),Mp_T([1 2],2,k),Mp_T([1 2],3,k),'r','linewidth',2)
        plot3(Mp_T([1 3],1,k),Mp_T([1 3],2,k),Mp_T([1 3],3,k),'g','linewidth',2)
        plot3(Mp_T([1 4],1,k),Mp_T([1 4],2,k),Mp_T([1 4],3,k),'b','linewidth',2)
        O(:,k+1)=Mp_T(1,1:3,k)';
    end
    plot3(O(1,:),O(2,:),O(3,:),'k-o','linewidth',1.5,'markerfacecolor','k')
    plot3(P.target_p(1),P.target_p(2),P.target_p(3),'mp','markersize',14,'markerfacecolor','m')
    axis equal; grid on
    axis([-0.3 1.0 -0.7 0.7 0 1.1])
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]')
    view(35,25)
    title(sprintf('%s: q = [%s] deg', nombres{n}, strtrim(sprintf('%.1f  ', rad2deg(q)))))
end

%% Transform tipo Denavit-Hartenberg
function T_ij=T_DH(a_ij,alpha_ij,s_i,theta_i)
T_ij =[               cos(theta_i),              -sin(theta_i),              0,               a_ij;
        cos(alpha_ij)*sin(theta_i), cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -s_i*sin(alpha_ij);
        sin(alpha_ij)*sin(theta_i), sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  s_i*cos(alpha_ij);
                          0,                          0,              0,                  1];
end

%% Pose ROS (posicion + cuaternion [x y z w]) -> matriz homogenea
function T=pose2T(p,q_xyzw)
qw=q_xyzw([4 1 2 3]); qw=qw(:)'/norm(qw);   % MATLAB usa [w x y z]
T=[quat2rotm(qw) p(:); 0 0 0 1];
end

%% Error de posicion [mm] y de orientacion [deg] entre dos matrices homogeneas
function [ep,ea]=errores(A,B)
ep=1000*norm(A(1:3,4)-B(1:3,4));
ea=rad2deg(acos(max(-1,min(1,(trace(A(1:3,1:3)'*B(1:3,1:3))-1)/2))));
end
