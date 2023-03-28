% load parameter file
paramTwoLinkPlanarRobot

% simulate
options=odeset('RelTol',1e-09,'AbsTol',1e-09);
% without this line that controls accuracy of your numerical solution, your
% energy will not come out to be 'very' constant 
%
tSim=15;% Pick some simulation time
theta10 = -pi/2;
theta20 = 0; 
dtheta10 = 0;
dtheta20 = 0;
x0=[theta10;theta20;dtheta10;dtheta20];
[tList,xList]=ode45(@odeTwoLinkPlanarRobot,[0 tSim],x0,options);
%%
% Postprocess - plot system trajectories
theta1List=xList(:,1);theta2List=xList(:,2);
dtheta1List=xList(:,3);dtheta2List=xList(:,4);

%%

% VELOCITIES OF THE CENTER OF MASSES

vC1_0List=zeros(length(theta1List),3);
vC2_0List=zeros(length(theta1List),3);
PC1_0List=zeros(length(theta1List),3);% CENTER OF MASS LOCATION Link # 2
PA_0List=zeros(length(theta1List),3); % END POINT OF LINK # 1
PC2_0List=zeros(length(theta1List),3);% CENTER OF MASS LOCATION Link # 2
PB_0List=zeros(length(theta1List),3); % END POINT OF LINK # 2
for i = 1:length(theta1List)
    vC1_0List(i,:) = [-dtheta1List(i)*r1*sin(theta1List(i)); dtheta1List(i)*r1*cos(theta1List(i)); 0]';
    vC2_0List(i,:) = [- dtheta1List(i)*(r2*sin(theta1List(i) + theta2List(i)) + L1*sin(theta1List(i))) - dtheta2List(i)*r2*sin(theta1List(i) + theta2List(i)); dtheta1List(i)*(r2*cos(theta1List(i) + theta2List(i)) + L1*cos(theta1List(i))) + dtheta2List(i)*r2*cos(theta1List(i) + theta2List(i)); 0]';
    PC1_0List(i,:) = [r1*cos(theta1List(i)); r1*sin(theta1List(i)); 0]';% CENTER OF MASS LOCATION
    PA_0List(i,:) = [L1*cos(theta1List(i)); L1*sin(theta1List(i)); 0]';% END POINT OF LINK # 1
    PC2_0List(i,:) = PA_0List(i,:) + [r2*cos(theta1List(i)+theta2List(i)); r2*sin(theta1List(i)+theta2List(i)); 0]';% CENTER OF MASS LOCATION
    PB_0List(i,:) = PA_0List(i,:) + [L2*cos(theta1List(i)+theta2List(i)); L2*sin(theta1List(i)+theta2List(i)); 0]';% END POINT OF LINK # 2
end

vC1_0List = vC1_0List(:,1).^2+vC1_0List(:,2).^2+vC1_0List(:,3).^2;
vC2_0List = vC2_0List(:,1).^2+vC2_0List(:,2).^2+vC2_0List(:,3).^2;

%Total Energy Calculations
% Link1
TKE1 = (1/2)*m1*vC1_0List;
omega1 = dtheta1List;
RKE1 = (1/2)*I1*omega1.^2;
PE1 = m1*g*PC1_0List(:,2);

% Link2
TKE2 = (1/2)*m2*vC2_0List;
omega2 = dtheta1List + dtheta2List;
RKE2 = (1/2)*I2*omega2.^2;
PE2 = m2*g*PC2_0List(:,2);
SUM_E = TKE1+RKE1+PE1+TKE2+RKE2+PE2;

% End-Effector Positions Xe and Ye
Xe = PB_0List(:,1);
Ye = PB_0List(:,2);

figure(1)
plot(tList,SUM_E)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('Energy [J]')
%%
figure(2)
plot(tList,theta1List,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('\Theta1 [rads]')

figure(3)
plot(tList,theta2List,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('\Theta2 [rads]')

figure(4)
plot(tList,Xe,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('Xe [m]')

figure(5)
plot(tList,Ye,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('Ye [m]')

%%
figure(6)
plot(tList,omega1,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('\omega_1 [rads/s]')

figure(7)
plot(tList,omega2,'LineWidth',4)
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('Time [s]')
ylabel('\omega_2 [rads/s]')



%% Drawing the arm
O_origin = [0; 0];
P1_position = [L1*cos(theta1d); L1*sin(theta1d)];
P2_position = [L1*cos(theta1d)+L2*cos(theta1d+theta2d); ...
    L1*sin(theta1d)+L2*sin(theta1d+theta2d)];

figure(8);

h_OP1 = plot([O_origin(1) P1_position(1)],[O_origin(2) P1_position(2)],'linewidth',10); 
hold on; % so that the first line is not erased
h=gca;h.FontSize=30;h.LineWidth=2;
grid on 
xlabel('X_e [m]')
ylabel('Y_e [s]')
h_P1P2 = plot([P1_position(1) P2_position(1)],[P1_position(2) P2_position(2)],'linewidth',10);
axis equal; % so that lengths in x direction are the same as lengths in y direction
plot(O_origin(1),O_origin(2),'o','markersize',20,'markerfacecolor',[0 0 0])


% animation
theta1d = pi/4;
theta2d = pi/6;
filename = "testAnimated.gif";

figure(9);

for indexI = 1:length(tList)
    h_OP1 = plot([O_origin(1) PA_0List(indexI,1)],[O_origin(2) PA_0List(indexI,2)],'linewidth',10); 
    hold on; % so that the first line is not erased
    h_P1P2 = plot([PA_0List(indexI,1) PB_0List(indexI,1)],[PA_0List(indexI,2) PB_0List(indexI,2)],'linewidth',10);
    axis equal;  % so that lengths in x direction are the same as lengths in y direction
    xlim([-2 2]); ylim([-2 2]); % so that axis limits do not change
    plot(O_origin(1),O_origin(2),'o','markersize',20,'markerfacecolor',[0 0 0])
    xlabel('x'); ylabel('y');
    pause(0.01);
    %%%%%%%%%%%%%%%%
    hold off; % to erase the figure for the next loop
    
    frame = getframe(figure(9));
    im{indexI} = frame2im(frame);
    
    [A,map] = rgb2ind(im{indexI},256);
    if indexI == 1
        imwrite(A,map,filename,"gif","LoopCount",Inf,"DelayTime",1);
    else
        imwrite(A,map,filename,"gif","WriteMode","append","DelayTime",1);
    end
end




