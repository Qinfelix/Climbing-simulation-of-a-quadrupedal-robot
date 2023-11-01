function [energy] = f1_calculate_potential_energy(anchor_pos,robot_pos,setting)
%UNTITLED Summary of this function goes here
%%%% anchor_pos=[x10,y10;x20,y20]
%%%% robot_pos=[x,y,theta]
%%%% spring_para=[deltax,deltay;kx,ky]
%%%% rpbot_mass=mass(unit kg)
%   Detailed explanation goes here

theta=robot_pos(3);
Rotatation_Matrix=[cos(theta),sin(theta);-sin(theta),cos(theta)];
spring_orgin_pos=setting.spring_para.origin_pos;
stiffness=setting.spring_para.stiffness;

% rotate the coordinate to the robot-horitional-coordinate to get the
% elongation
anchor_pos=Rotatation_Matrix*anchor_pos';
spring_origin=Rotatation_Matrix*robot_pos(1,1:2)'+spring_orgin_pos';
origin_all=nan(2,setting.leg_num);
for leg_index = 1:setting.leg_num
    origin_all(:,leg_index)=spring_origin;
end
elongation=anchor_pos-origin_all;

%compute the energy
elastic=sum(0.5*stiffness*(elongation.*elongation),'all');
energy=elastic+setting.robot_mass*9.8*robot_pos(2);

end

