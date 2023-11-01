%Yue write this code to calcute the Jocabian matrix of 5-bar legs at 2023/08/07

clc;
clear;
L=[70,30,120,60,65,25];

% calculate based on the chain rule

% syms theta1 theta2
% x_c=L(5)+L(1)*cos(theta1);
% y_c=L(1)*sin(theta1);
% x_e=L(4)*cos(theta2);
% y_e=L(4)*sin(theta2);
% var1=[theta1,theta2];
% f1=[x_c,y_c,x_e,y_e];
% 
% J1=simplify(jacobian(f1,var1));
% 
% syms x_c y_c x_e y_e
% CE=norm([x_c-x_e,y_c-y_e]);
% EH=(L(3)^2+CE^2-L(2)^2)/(2*CE);
% x_h=x_e+EH/CE*(x_c-x_e);
% y_h=y_e+EH/CE*(y_c-y_e);
% k=(y_c-y_e)/CE;
% 
% f2=[x_h,y_h,k];
% var2=[x_c,y_c,x_e,y_e];
% 
% J2=simplify(jacobian(f2,var2));


















% calculate as a whole function expression --> too complex


syms theta1 theta2
% C=[L(5)+L(1)*cos(theta1),L(1)*sin(theta1)];
syms x_c y_c

C=[x_c,y_c];
E=[L(4)*cos(theta2),L(4)*sin(theta2)];
CE=norm(C-E);
alpha=atan((C(2)-E(2))/(C(1)-E(1)));
beta=acos((L(2)^2+CE^2-L(3)^2)/(2*CE*L(2)));
x=C(1)-(L(2)+L(6))/cos(beta-alpha);
y=C(2)+(L(2)+L(6))*sin(beta-alpha);

f=[x,y];
% vars=[theta1,theta2];
var2=[x_c,y_c];

J=jacobian(f,var2);

J_simplifed=simplify(J)


