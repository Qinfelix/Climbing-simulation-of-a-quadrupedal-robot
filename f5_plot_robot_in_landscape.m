function [] = f5_plot_robot_in_landscape(state_now,frame_i,para,landscape_plot)
%F5_PLOT_ROBOT_IN_LANDSCAPE Summary of this function goes here
%   Detailed explanation goes here

fig=figure;
set(fig,'Position',[100,100,1500,500])

%%% plot the robot point in the landscape
subplot(1,2,1);
imagesc(landscape_plot')
axis xy
xlabel('x/mm')
ylabel('z/mm')
% axis off
colormap(fire)
colorbar
hold on
scatter(state_now.body.x,state_now.body.z,100,'filled','MarkerFaceColor','white')
for i =1:para.anchor_num
    scatter(para.anchor_pos(1,i),para.anchor_pos(2,i),100,'d','MarkerFaceColor','black')
end
title('Potential Energy Landscape(J)','FontSize',14)



%%% plot the robot posture
subplot(1,2,2)
theta=state_now.body.theta;
angle_robotsize=0.1;
x_plot=[state_now.body.x+para.length*cos(theta-angle_robotsize),...
    state_now.body.x+para.length*cos(theta+angle_robotsize),...
    state_now.body.x-para.length*cos(theta-angle_robotsize),...
    state_now.body.x-para.length*cos(theta+angle_robotsize)];
z_plot=[state_now.body.z+para.length*sin(theta-angle_robotsize),...
    state_now.body.z+para.length*sin(theta+angle_robotsize),...
    state_now.body.z-para.length*sin(theta-angle_robotsize),...
    state_now.body.z-para.length*sin(theta+angle_robotsize)];
patch(x_plot,z_plot,'blue')
hold on

robot_pos=[state_now.body.x;state_now.body.z];

for leg_i=1:para.leg_num
    leg_orgin_pos_to_robot=[para.leg_origin_pos(1,leg_i);-para.leg_origin_pos(2,leg_i)];

    R_matrix=[cos(theta),-sin(theta);sin(theta),cos(theta)];
    leg_orgin_pos_to_robot=R_matrix*leg_orgin_pos_to_robot;
    origin_pos=robot_pos+leg_orgin_pos_to_robot;
%     scatter(origin_pos(1),origin_pos(2),'filled','MarkerFaceColor','blue')
    plot([robot_pos(1),origin_pos(1)],[robot_pos(2),origin_pos(2)],'--','LineWidth',2)

    %plot the physical leg
    leg_connection_pos=robot_pos + R_matrix*para.leg_connector_pos(:,leg_i);
    anchor_pos=para.anchor_pos(:,para.leg_standing_anchor(leg_i));
    plot([leg_connection_pos(1),anchor_pos(1)],[leg_connection_pos(2),anchor_pos(2)],'-','LineWidth',3,'Color','black')
    hold on

end

for i =1:para.anchor_num
    scatter(para.anchor_pos(1,i),para.anchor_pos(2,i),100,'filled','MarkerFaceColor','black')
end
axis([0,500,0,300])
title('Robot Posture in Physical World','FontSize',14)

savefilename=['simulation_video_2/with_landscape_robot_posture_',num2str(frame_i),'.png'];
saveas(fig,savefilename,'png')
close(fig);

end

