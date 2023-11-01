function [force_from_anchors] = f2_controller_rule(state,para)
%F2_CONTROLLER_RULE Summary of this function goes here
%   Detailed explanation goes here

%rotate the coordinate to make x consistent with x_robot
rotation_angle=-state.body.theta;
R_matrix=[cos(rotation_angle),-sin(rotation_angle);sin(rotation_angle),cos(rotation_angle)];
R_matrix_reverse=[cos(-rotation_angle),-sin(-rotation_angle);sin(-rotation_angle),cos(-rotation_angle)];
robot_pos=R_matrix*[state.body.x;state.body.z];
force_from_anchors=nan(2,para.leg_num);
for leg_i = 1:para.leg_num
    %get the anchor pos to robot
    anchor_i=para.leg_standing_anchor(leg_i);
    anchor_pos_to_robot=R_matrix*para.anchor_pos(:,anchor_i);
    anchor_pos_to_robot(1)=anchor_pos_to_robot(1)-robot_pos(1);
    %the y_robot is the reverse direction of z
    anchor_pos_to_robot(2)=robot_pos(2)-anchor_pos_to_robot(2);

    displacement=anchor_pos_to_robot-para.leg_origin_pos(:,leg_i);

    force_in_robot_coor=displacement.*para.leg_stiffness(:,leg_i);

    %transfer the force to the anchor cooridinate
    force_in_robot_coor(2)=-force_in_robot_coor(2);%flip the y_robot
    force_in_anchor_coor=R_matrix_reverse*force_in_robot_coor;


    firction=-[state.body.x_vel,state.body.z_vel].*para.friction_k;
    force_from_anchors(:,leg_i)=force_in_anchor_coor+firction';



end



end

