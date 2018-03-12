link1=createLink(0,0,0,[],[0;0;0],0,[0 0 0;0 0 0;0 0 0]);
link2=createLink(30,0,0,[],[15;0;0],2.064,[167.7 0 0;0 167.7 0;0 0 25.8]);
link3=createLink(25,0,0,[],[12.5;0;0],1.720,[96.46 0 0;0 96.46 0;0 0 13.76]);
link4=createLink(9.25,0,0,0,[4.625;0;0],0.636,[5.966 0 0;0 5.966 0;0 0 2.862]);
linkList=[link1 link2 link3 link4];

paramListG = [0.00001;0;0;0];

figure
axis([-70 70 -70 70])
daspect([1 1 1])

%% Plot Initial Configuration of the Arm 
clf;
for i = 1:1:length(paramListG)
    H(:,:,i) = dhFwdKine(linkList(1:i), paramListG(1:i));
end

for i = 1:1:length(paramListG)-1
    
    hold on;
    grid on;
    %plot3([H(1,4,i) H(1,4,i+1)],[H(2,4,i) H(2,4,i+1)],[H(3,4,i) H(3,4,i+1)])
    
    % Plot the arms and the hand
    PLT = plot([H(1,4,i) H(1,4,i+1)],[H(2,4,i) H(2,4,i+1)],'LineWidth',5);
    uistack(PLT, 'bottom')
    
    % Plot the elbow, and wrist joints
    CIR = scatter( H(1,4,i+1),H(2,4,i+1),'o','LineWidth',5,'LineWidth',4,'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .5 .5]);
    uistack(CIR, 'bottom')
    
    % Plot the shoulder joint
    ORG = scatter( 0,0,'p','LineWidth',5,'LineWidth',4,'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .5 .5]);
    
    % Add coordinates of the joint s
    str = ['\uparrow (',num2str(H(1,4,i+1)),' , ',num2str(H(2,4,i+1)),')'];
    TXT = text(H(1,4,i+1),H(2,4,i+1)-5*i,str);
    uistack(TXT, 'top')
    
    % Set plot specs
    axis([-70 70 -70 70])
    daspect([1 1 1])
end
%pause(0.5)



%% Inverse Dynamics and Animation

while(1)
    prompt = {'Enter Hand Angle (from the X axis):','Enter Movement Duration Time:','Enter Object Mass in hand (kg):'};
    dlg_title = 'Input Conditions';
    num_lines = 1;
    input = inputdlg(prompt,dlg_title,num_lines);
    
    % Input Hand Angle Degree
    deg = str2double(input{1,:});
    
    % Input Movement Time Duration
    time = str2double(input{2,:});
    
    % Input Mass in hand
    mass = str2double(input{3,:});
    
    % Point the Desired End Effector Position
    h = impoint;
    pos = getPosition(h);
    
    % Check if the point exceeds the max reach
    end_angle = atan(pos(2)/pos(1))*180./pi;
    diff = abs(end_angle -deg);
    if diff > 180
        diff = diff - 180;
    end
    while(-(diff/90)*9.25+9.25+55 < sqrt((pos(1))^2+(pos(2))^2))
        
        str = ['\leftarrow Exceeds Maximum Reach'];
        TXT_warn = text(pos(1),pos(2),str);
        pause(1)
        delete(TXT_warn);
        h = impoint;
        pos = getPosition(h);
    end
    
    % Setup desTransform
    desTransform = [ cos(deg.*pi./180) -sin(deg.*pi./180) 0 pos(1);...
                    sin(deg.*pi./180) cos(deg.*pi./180) 0 pos(2);...
                    0 0 1 0;...
                    0 0 0 1];
                
    % Inverse Dynamics
    [paramList, error] = dhInvKine (linkList, desTransform, paramListG);
    
   
    
    % 3rd Degree Polynomial Path Planning
    
    a_1 = 6*(-2*(paramList(1)-paramListG(1))/(time^3));
    b_1 = -2*(-3*(paramList(1)-paramListG(1))/(time^2));
    a_2 = 6*(-2*(paramList(2)-paramListG(2))/(time^3));
    b_2 = -2*(-3*(paramList(2)-paramListG(2))/(time^2));
    a_3 = 6*(-2*(paramList(3)-paramListG(3))/(time^3));
    b_3 = -2*(-3*(paramList(3)-paramListG(3))/(time^2));
    a_4 = 6*(-2*(paramList(4)-paramListG(4))/(time^3));
    b_4 = -2*(-3*(paramList(4)-paramListG(4))/(time^2));
    
    theta_prev = paramListG;
    vel_prev = [0;0;0;0];
    
    
for t = 0:(time/30):time
    clf;
    theta1 = (1./6)*a_1*(t^3)+(1./2)*b_1*(t^2)+paramListG(1);
    theta2 = (1./6)*a_2*(t^3)+(1./2)*b_2*(t^2)+paramListG(2);
    theta3 = (1./6)*a_3*(t^3)+(1./2)*b_3*(t^2)+paramListG(3);
    theta4 = (1./6)*a_4*(t^3)+(1./2)*b_4*(t^2)+paramListG(4);
    
    theta_curr = [theta1;theta2;theta3;theta4];
    %% Velocity Calc
    
    vel_curr = (theta_curr - theta_prev)/(time./30);
    %% Accerleration Calc
    
    acc_curr = (vel_curr - vel_prev)/(time/30);  
    %% Torque Calc
    baseDynamics.linA = [0;0;0];
    baseDynamics.angV = [0;0;0];
    baseDynamics.angA = [0;0;0];
    z_tor = -mass*9.8*(link2.a*cos(theta1)+link3.a*cos(theta2)+link4.a*(theta3));
    endEffectorWrench = [0;-mass*9.8;0;0;0;z_tor];
    gravityDirection = [0;0;-1];
    [jointTorques,Jv,JvDot]=newtonEuler(linkList,theta_curr,vel_curr,acc_curr,baseDynamics,endEffectorWrench,gravityDirection);
    %% Animation Calc
    
    hold on;
    grid on;
    
    % Plot the Mass on hand
    str = ['Mass on Hand = ',num2str(mass),' kg'];
    TXT_mass = text(H(1,4,4)-5,H(2,4,4)+18,str);
    uistack(TXT_mass, 'top')
    p1 = [H(1,4,4) H(2,4,4)+2];
    p2 = [H(1,4,4) H(2,4,4)+15];
    dp = p1 - p2;
    quiver(p2(1),p2(2),dp(1),dp(2),0,'MaxHeadSize',2)
    
    for i = 1:1:length(theta_curr)
        H(:,:,i) = dhFwdKine(linkList(1:i), theta_curr(1:i));
    end

    for i = 1:1:length(theta_curr)-1
        
        %plot3([H(1,4,i) H(1,4,i+1)],[H(2,4,i) H(2,4,i+1)],[H(3,4,i) H(3,4,i+1)])
        
        % Plot the arms and the hand
        PLT = plot([H(1,4,i) H(1,4,i+1)],[H(2,4,i) H(2,4,i+1)],'LineWidth',5);
        uistack(PLT, 'bottom')
        
        % Plot the elbow, and wrist joints
        CIR = scatter( H(1,4,i+1),H(2,4,i+1),'o','LineWidth',4,'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .5 .5]);
        uistack(CIR, 'bottom')
        
        % Plot the shoulder joint
        ORG = scatter( 0,0,'p','LineWidth',5,'LineWidth',4,'MarkerEdgeColor',[0 .5 .5],'MarkerFaceColor',[0 .5 .5]);
        
        % Add coordinates of the joints
        str = ['\uparrow (',num2str(H(1,4,i+1)),' , ',num2str(H(2,4,i+1)),')'];
        TXT_coor = text(H(1,4,i+1),H(2,4,i+1)-2,str);
        uistack(TXT_coor, 'top')
        
        % Add coordinates of the joints
        str = [num2str(180*theta_curr(i)/pi),'\circ \rightarrow',];
        TXT_ang = text(H(1,4,i)-20,H(2,4,i),str);
        uistack(TXT_ang, 'top')
        
        % Display the velocity, accerleration and Torque of the joints
        str = ['Angular Velocity ',num2str(i),' = ',num2str(vel_curr(i)*180/pi),' degree/sec'];
        TXT_vel = text(-170,60-5*i,str,'FontSize',12,'FontWeight','bold');
        uistack(TXT_vel, 'top')
        
        str = ['Angular Acceleration ',num2str(i),' = ',num2str(acc_curr(i)*180/pi),' degree/(sec)^2'];
        TXT_acc = text(-170,10-5*(i),str,'FontSize',12,'FontWeight','bold');
        uistack(TXT_acc, 'top')
        
        str = ['Torque ',num2str(i),' = ',num2str(jointTorques(i)/100),' N-m'];
        TXT_tor = text(-170,-40-5*(i),str,'FontSize',12,'FontWeight','bold');
        uistack(TXT_tor, 'top')
        
        % Set plot specs
        axis([-70 70 -70 70])
        daspect([1 1 1])
    end
    theta_prev = theta_curr;
    vel_prev = vel_curr;
    pause(time/30)
   
end
           paramListG = paramList; 
end
