% newtonEuler - Computes the inverse dynamics of a serial link manipulator 
%               and provides the velocity jacobian and its rate of change.
%
%   [jointTorques, Jv, JvDot] = newtonEuler(linkList, paramList, paramListDot, paramListDDot,baseDynamics, endEffectorWrench,gravityDirection) 
%
%       With the input of the links consist in the array linkList, the
%       current states of the joint variables consist in the array
%       paramList, the current rate of change of joint variables in the
%       array paramListDot, and the rate of chenge of that in
%       paramListDDot, the base dynamics of the machanical structure, the
%       force and torque applied on the end effector of the machanical
%       structure, and the gravitational pull direction, this function
%       returns the torque of each joint, the velocity jacobian, and the
%       time derivative of the jacobian.
%
%   linkList = the array consisting all the link structures, every
%              structure consists all the information need for the link
%   paramList = the array that consists the current joint variable
%               positions
%   paramListDot = the array that consists the rate of change of the joint
%                  variables
%   baseDynamics = the the angular velocity and acceleration of the base 
%                  frame expressed in the base frame
%   endEffectorWrench = the externally applied force and torque on the last 
%                       frame expressed in the last frame.
%   gravityDirection = the direction of gravity expressed in the base frame
%   jointTorques = returns the torque of each joint
%   Jv = returns the velocity jacobian of the system
%   JvDot = returns the time derivative of the velocity jacobian
%
% Michael Cheng
% CWID: 10820067
% MENG 544: Robot Mechanics: Kinematics, Dynamics, and Control
% 11/13/2016

function [jointTorques, Jv, JvDot] = newtonEuler( linkList,paramList, paramListDot, paramListDDot,baseDynamics, endEffectorWrench,gravityDirection )

%Prellocating arrays


A=length(linkList);
Pc = zeros(3,1,A);
m = zeros(A,1);
I = zeros(3,3,A);
FwdLinkTransforms = zeros(4,4,A);
InvLinkTransforms = zeros(4,4,A);
FwdR = zeros(3,3,A);
InvR = zeros(3,3,A);
FwdP = zeros(3,1,A);
InvP = zeros(3,1,A);
w = zeros(3,1,A+1);
v = zeros(3,1,A+1);
w_dot = zeros(3,1,A+1);
v_dot = zeros(3,1,A+1);
vc_dot = zeros(3,1,A);
F = zeros(3,1,A);
N = zeros(3,1,A);
f = zeros(3,1,A+1);
n_inward = zeros(3,1,A+1);
torque = zeros(A,1);
JvDot = zeros(6,A);

%Inputing the base dynamics
w(:,:,1) = baseDynamics.angV;
w_dot(:,:,1) = baseDynamics.angA;
v_dot(:,:,1) = baseDynamics.linA + 9.8*gravityDirection;

%Putting paramList values in to linkList
z=1;
for n = 1:1:A
    if linkList(n).isRotary == 1
        linkList(n).theta=paramList(z);
        z=z+1;
    elseif linkList(n).isRotary == 0
        linkList(n).d = paramList(z);
        z=z+1;
    else
    end
end

%Taking the centOfMass from each link and putting them into a list: Pc
for n = 1:1:A
    Pc(:,:,n) = linkList(n).com;
end

%Taking the mass from each link and putting them into a list: m
for n = 1:1:A
    m(n) = linkList(n).mass;
end

%Taking the inertia from each link and putting them into a list: I
for n = 1:1:A
    I(:,:,n) = linkList(n).inertia;
end

%Getting forward transform list: FwdLinkTransforms
for n = 1:1:A
    FwdLinkTransforms(:,:,n) = dhTransform(linkList(n).a,linkList(n).d,linkList(n).alpha,linkList(n).theta);
end

%Getting inverse transform list: InvLinkTransforms
for n = 1:1:A
    InvLinkTransforms(:,:,n) = inv(FwdLinkTransforms(:,:,n));
end

%Getting forward rotation matrix list: FwdR
for n = 1:1:A
    FwdR(:,:,n) = FwdLinkTransforms(1:3,1:3,n);
end

%Getting inverse rotation matrix list: InvR
for n = 1:1:A
    InvR(:,:,n) = InvLinkTransforms(1:3,1:3,n);
end

%Getting forward position matrix list: FwdP
for n = 1:1:A
    FwdP(:,:,n) = FwdLinkTransforms(1:3,4,n);
end

%Getting inverse position matrix list: InvP
for n = 1:1:A
    InvP(:,:,n) = InvLinkTransforms(1:3,4,n);
end

%find w list: w
for i = 0:1:A-1
    w(:,:,(i+1)+1) = InvR(:,:,(i)+1)*w(:,:,(i)+1) + paramListDot(i+1)*[0;0;1];
end

%find w_dot list: w_dot
for i = 0:1:A-1
    w_dot(:,:,(i+1)+1) = InvR(:,:,(i)+1)*w_dot(:,:,(i)+1) + cpMatrix(InvR(:,:,(i)+1)*w(:,:,(i)+1))*paramListDot(i+1)*[0;0;1] + paramListDDot(i+1)*[0;0;1];
end

%find v list: v
for i = 0:1:A-1
    v(:,:,(i+1)+1) = InvR(:,:,(i)+1)*(v(:,:,(i)+1) + cross(w(:,:,(i)+1),FwdP(:,:,(i)+1)));
end

%find v_dot list: v_dot
for i = 0:1:A-1
    v_dot(:,:,(i+1)+1) = InvR(:,:,(i)+1)*(cross(w_dot(:,:,(i)+1),FwdP(:,:,(i)+1)) + cpMatrix(w(:,:,(i)+1))*cpMatrix(w(:,:,(i)+1))*FwdP(:,:,(i)+1) + v_dot(:,:,(i)+1)); 
end

%find vc_dot list: vc_dot
for i = 0:1:A-1
    vc_dot(:,:,(i)+1) = cross(w_dot(:,:,(i+1)+1),Pc(:,:,i+1)) + cross(w(:,:,(i+1)+1),cross(w(:,:,(i+1)+1),Pc(:,:,(i)+1))) + v_dot(:,:,(i+1)+1); 
end

%find F list: F
for i = 0:1:A-1
    F(:,:,(i)+1) = m((i)+1)*vc_dot(:,:,(i)+1);
end

%find N list: N
for i = 0:1:A-1
    N(:,:,(i)+1) = I(:,:,(i)+1)*w_dot(:,:,(i+1)+1) + cpMatrix(w(:,:,(i+1)+1))*(I(:,:,(i)+1)*w(:,:,(i+1)+1));
end

%there will be a mismatch for FwdR and FwdP, so add some modifications
FwdR2(:,:,1:A)=FwdR(:,:,1:A);
FwdR2(:,:,A+1)=FwdR(:,:,A);
FwdP2=zeros(3,1,A+1);
FwdP2(:,:,1:A)=FwdP(:,:,1:A);
FwdP2(:,:,A+1)=[0;0;0];

%find f list: f
for i = A:-1:1
    f(:,:,i) = FwdR2(:,:,i+1)*f(:,:,i+1) + F(:,:,i);
end

%find n list: n
for i = A:-1:1
    n_inward(:,:,i) = N(:,:,i) + FwdR2(:,:,i+1)*n_inward(:,:,i+1) + cpMatrix(Pc(:,:,i))*F(:,:,i) + cpMatrix(FwdP2(:,:,i+1))*FwdR2(:,:,i+1)*f(:,:,i+1);
end
%find torque list: torque
for i = A:-1:1
    torque(i) = n_inward(:,:,i)'*[0;0;1];
end

%Calculate Jv 
Jv = (velocityJacobian(linkList, paramList));

%Calculate the joint torques
t_endEffector = Jv'*endEffectorWrench;
jointTorques = torque + t_endEffector;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Compute JvDot

T0_N = dhFwdKine(linkList,paramList);
d0_N = T0_N(1:3,4);
%Vtool = Jv*paramListDot;
Vtool = T0_N(1:3,1:3)*v(:,:,A+1);
T_cur = eye(4,4);


for q=1:1:A
    T_cur = T_cur*FwdLinkTransforms(:,:,q);
    Ti_N = T_cur;
    z = Ti_N(1:3,3);
    d = Ti_N(1:3,4);
    W = Ti_N(1:3,1:3)*w(:,:,q+1);
    V = Ti_N(1:3,1:3)*v(:,:,q+1);
    JvDot(:,q) = [cross(cross(W,z),(d0_N-d))+cross(z,Vtool-V);cross(W,z)];       
end

