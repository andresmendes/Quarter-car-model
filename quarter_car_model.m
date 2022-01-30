%% Quarter car model
% Simulation and animation of a quarter car model.
%
%%
clear ; close all ; clc

%% Parameters

% Vehicle
M   = 1500;                         % Sprung mass                   [kg]
m   = 150;                          % Unsprung mass                 [kg]
Ks  = 48000;                        % Spring constant suspension    [N/m]
Kt  = 200000;                       % Spring constant tire          [N/m]
Cs  = 10000;                        % Damping constant suspension   [N.s/m]

% Animation model
L0_s    = 0.8;                      % Spring relaxed suspension     [m]
L0_u    = 0.6;                      % Spring relaxed tire           [m]
h_s     = 0.4;                      % Height of the sprung block    [m]
h_u     = 0.2;                      % Height of the unsprung block  [m]
a       = 0.8;                      % Width of the blocks           [m]
l_win   = 2.2;                      % Length window analysis        [m]

% Video
playback_speed = 0.2;               % Speed of playback
tF      = 2;                        % Final time                    [s]
fR      = 30/playback_speed;        % Frame rate                    [fps]
dt      = 1/fR;                     % Time resolution               [s]
time    = linspace(0,tF,tF*fR);     % Time                          [s]

%% Road
% Stretch 1
x_r_1_total = 1.1;                  % Distance of the first stretch [m]
dx_r_1 = 0.1;                       % resolution                    [m]
x_r_1 = 0:dx_r_1:x_r_1_total;
z_r_1 = zeros(1,length(x_r_1));
% Stretch 2
R_r = 0.15;                         % Radius                        [m]
th_r = 0:0.01:pi;
x_r_2 = -R_r*cos(th_r) + x_r_1_total+R_r;
z_r_2 = R_r*sin(th_r);
% Stretch 3
x_r_3_total = 5;                    % Distance of the last stretch  [m]
dx_r_2 = 0.1;                       % resolution                    [m]
x_r_3 = x_r_1_total+2*R_r:dx_r_2:x_r_1_total+2*R_r+x_r_3_total;
z_r_3 = zeros(1,length(x_r_3));

% Concatenating 
X_r = [x_r_1 x_r_2(2:end) x_r_3(2:end)];
Z_r = [z_r_1 z_r_2(2:end) z_r_3(2:end)];

figure
hold on ; box on ; grid on ; axis equal
plot(x_r_1,z_r_1)
plot(x_r_2,z_r_2)
plot(x_r_3,z_r_3)

figure
hold on ; box on ; grid on ; axis equal
plot(X_r,Z_r,'k','LineWidth',2)

%% Simulation

%  State space model
A = [ 0               1         0       0       ;
      -(Ks+Kt)/m      -Cs/m     Ks/m    Cs/m    ;
      0               0         0       1       ;
      Ks/M            Cs/M      -Ks/M   -Cs/M   ];
B = [ 0     ;
      Kt/m  ;
      0     ;
      0     ];
C = [ 1 0 0 0 ; 
      0 0 1 0 ];
D = [0 ; 0];

sys = ss(A,B,C,D);

% Input
vel = 2;                            % Longitudinal speed of the car [m/s]
lon_pos = vel*time;                 % Longitudinal position of the car [m]
u_vet = interp1(X_r,Z_r,lon_pos)';

figure
hold on ; grid on ; box on
plot(time,u_vet)
legend('u')

[y,time,x] = lsim(sys,u_vet,time);

% Sprung mass absolute vertical position (lower center point)
z_s = y(:,2) + L0_u + L0_s; 
% Unsprung mass absolute vertical position (lower center point)
z_u = y(:,1) + L0_u; 

figure
hold on ; grid on ; box on
plot(time,z_s)
plot(time,z_u)
xlabel('Time [s]')
ylabel('Vertical coordinate [m]')
legend('z_s','z_u')

%% Animation

color = cool(6); % Colormap

figure
% set(gcf,'Position',[50 50 1280 720])  % YouTube: 720p
% set(gcf,'Position',[50 50 854 480])   % YouTube: 480p
set(gcf,'Position',[50 50 640 640])     % Social

% Create and open video writer object
v = VideoWriter('quarter_car_model.mp4','MPEG-4');
v.Quality   = 100;
% v.FrameRate = fR;
open(v);

for i=1:length(time)
    
    cla
    
    % Instant position
    x_inst = vel*time(i);
    
    % Road passing by:
    set(gca,'xlim',[x_inst-l_win/2    x_inst+l_win/2],'ylim',[-0.1 -0.1+l_win])
    hold on ; grid on ; box on %; axis equal (Dont work well. It drifts vertically)
    plot([-10 X_r],[0 Z_r],'k','LineWidth',3)
    
    set(gca,'FontName','Verdana','FontSize',16)
    title(["Quarter car model",strcat('Time=',num2str(time(i),'%.3f'),' s (Playback speed=',num2str(playback_speed),')')])
    
    % Sprung mass plot
    fill([x_inst-a/2 x_inst+a/2 x_inst+a/2 x_inst-a/2],[z_s(i) z_s(i) z_s(i)+h_s z_s(i)+h_s],color(6,:),'LineWidth',2)
    
    % Unsprung mass plot
    fill([x_inst-a/2 x_inst+a/2 x_inst+a/2 x_inst-a/2],[z_u(i) z_u(i) z_u(i)+h_u z_u(i)+h_u],color(2,:),'LineWidth',2)
    
    % Spring
    plotSpring(L0_u,L0_s,h_u,u_vet,z_s,z_u,i,x_inst)
    
    % Damper
    plotDamper(L0_s,h_u,z_s,z_u,i,x_inst)
   
    % Tire
    plot(x_inst,u_vet(i),'ko','MarkerFacecolor','k','MarkerSize',10)
    
    xlabel('x [m]')
    ylabel('z [m]')
    
    frame = getframe(gcf);
    writeVideo(v,frame);
    
end

close(v);

function plotSpring(L0_u,L0_s,h_u,u_vet,z_s,z_u,i,x_inst)

    rodPct      = 0.11;     % Length rod percentage of total L0 
    springPct   = 1/3;      % Spring pitch percentage of total gap
    spring_wid  = 3;        % Spring line width
    
    % Spring 1 and 2 length without rods
    L_s = (z_s - (z_u+h_u)) - 2*rodPct*L0_s;
    L_u = (z_u - u_vet)     - 2*rodPct*L0_u;
    
    % Spring sprung suspension geometry
    c_s = x_inst-0.2;       % Longitudinal position
    w_s = 0.1;              % Width
    
    % Spring unsprung tire geometry 
    c_u = x_inst;           % Longitudinal position
    w_u = 0.1;              % Width

    % Spring unsprung tire 
    spring_u_X = [ 
                c_u                                             % Start
                c_u                                             % rod
                c_u+w_u                                         % Part 1   
                c_u-w_u                                         % Part 2
                c_u+w_u                                         % Part 3
                c_u-w_u                                         % Part 4
                c_u+w_u                                         % Part 5
                c_u-w_u                                         % Part 6
                c_u                                             % Part 7
                c_u                                             % rod/End
                ];
    
	spring_u_Z = [ 
                u_vet(i)                                        % Start
                u_vet(i)+  rodPct*L0_u                          % rod
                u_vet(i)+  rodPct*L0_u                          % Part 1 
                u_vet(i)+  rodPct*L0_u+  springPct*L_u(i)       % Part 2
                u_vet(i)+  rodPct*L0_u+  springPct*L_u(i)       % Part 3
                u_vet(i)+  rodPct*L0_u+2*springPct*L_u(i)       % Part 4
                u_vet(i)+  rodPct*L0_u+2*springPct*L_u(i)       % Part 5
                u_vet(i)+  rodPct*L0_u+3*springPct*L_u(i)       % Part 6
                u_vet(i)+  rodPct*L0_u+3*springPct*L_u(i)       % Part 7
                u_vet(i)+2*rodPct*L0_u+3*springPct*L_u(i)       % rod/End
               ]; 
           
    % Spring sprung suspension
    spring_s_X = [ 
                c_s                                             % Start
                c_s                                             % rod
                c_s+w_s                                         % Part 1   
                c_s-w_s                                         % Part 2
                c_s+w_s                                         % Part 3
                c_s-w_s                                         % Part 4
                c_s+w_s                                         % Part 5
                c_s-w_s                                         % Part 6
                c_s                                             % Part 7
                c_s                                             % rod/End
                ];
    
	spring_s_Z = [ 
                z_u(i)+h_u                                      % Start
                z_u(i)+h_u +   rodPct*L0_s                      % rod
                z_u(i)+h_u +   rodPct*L0_s                      % Part 1 
                z_u(i)+h_u +   rodPct*L0_s +   springPct*L_s(i) % Part 2
                z_u(i)+h_u +   rodPct*L0_s +   springPct*L_s(i) % Part 3
                z_u(i)+h_u +   rodPct*L0_s + 2*springPct*L_s(i) % Part 4
                z_u(i)+h_u +   rodPct*L0_s + 2*springPct*L_s(i) % Part 5
                z_u(i)+h_u +   rodPct*L0_s + 3*springPct*L_s(i) % Part 6
                z_u(i)+h_u +   rodPct*L0_s + 3*springPct*L_s(i) % Part 7
                z_u(i)+h_u + 2*rodPct*L0_s + 3*springPct*L_s(i) % rod/End
               ];

    % PLOT
    plot(spring_u_X,spring_u_Z,'k','LineWidth',spring_wid)
    plot(spring_s_X,spring_s_Z,'k','LineWidth',spring_wid)
        
end

function plotDamper(L0_2,h1,z_s,z_u,i,x_inst)
 
    rodLowerPct = 0.1;      % Length lower rod percentage of total gap 
    rodUpperPct = 0.4;      % Length upper rod percentage of total gap
    cylinderPct = 0.4;      % Length cylinder porcentagem of total gap
    damper_line_wid  = 3;   % Damper line width
    
    % Damper geometry
    c = 0.2+x_inst;         % Longitudinal position
    w= 0.05;                % Width

    % rod attached to unsprung mass
    rod_1_X = [c c];
    rod_1_Z = [z_u+h1 z_u+h1+rodLowerPct*L0_2];
    
    % Damper base cylinder - rod - base 
    c_X =   [   
                c-w
                c-w
                c+w
                c+w
            ];

    c_Z =   [
                z_u(i) + h1 + rodLowerPct*L0_2 + cylinderPct*L0_2
                z_u(i) + h1 + rodLowerPct*L0_2 
                z_u(i) + h1 + rodLowerPct*L0_2 
                z_u(i) + h1 + rodLowerPct*L0_2 + cylinderPct*L0_2
            ];
    
    % rod attached to sprung mass
    rod2X = [c c];
    rod2Z = [z_s z_s-rodUpperPct*L0_2];
    % Piston inside cylinder
    pistonX = [c-0.8*w c+0.8*w];
    pistonZ = [z_s-rodUpperPct*L0_2 z_s-rodUpperPct*L0_2];
    
    % Iteration values
    rod1Zval = rod_1_Z(i,:);
    rod2Zval = rod2Z(i,:);
    pistonZVal = pistonZ(i,:);

    % PLOT
    % rods
    plot(rod_1_X,rod1Zval,'k','LineWidth',damper_line_wid)
    plot(rod2X,rod2Zval,'k','LineWidth',damper_line_wid)
    % Damper parts
    plot(pistonX,pistonZVal,'k','LineWidth',damper_line_wid)
    plot(c_X,c_Z,'k','LineWidth',damper_line_wid)

end
