function [] = f4_plot_robot_posture(state_now,frame_i,para)

fig=figure;
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
f1=patch(x_plot,z_plot,'blue');
hold on

robot_pos=[state_now.body.x;state_now.body.z];

for leg_i=1:para.leg_num
    %plot the virtual spring
    leg_orgin_pos_to_robot=[para.leg_origin_pos(1,leg_i);-para.leg_origin_pos(2,leg_i)];
    leg_orgin_connector=[para.leg_origin_pos(1,leg_i);0];
    R_matrix=[cos(theta),-sin(theta);sin(theta),cos(theta)];
    leg_orgin_pos_to_robot=R_matrix*leg_orgin_pos_to_robot;
    leg_orgin_connector_to_robot=R_matrix*leg_orgin_connector;
    origin_pos=robot_pos+leg_orgin_pos_to_robot;
    origin_connector_pos=robot_pos+leg_orgin_connector_to_robot;
%     scatter(origin_pos(1),origin_pos(2),'filled','MarkerFaceColor','blue')
    f2=plot([origin_connector_pos(1),origin_pos(1)],[origin_connector_pos(2),origin_pos(2)],'--','LineWidth',2,'Color','red');
    hold on
    %plot the physical leg
    leg_connection_pos=robot_pos + R_matrix*para.leg_connector_pos(:,leg_i);
    anchor_pos=para.anchor_pos(:,para.leg_standing_anchor(leg_i));
    f3=plot([leg_connection_pos(1),anchor_pos(1)],[leg_connection_pos(2),anchor_pos(2)],'-','LineWidth',3,'Color','black');
    hold on
end

for i =1:para.anchor_num
    f4=scatter(para.anchor_pos(1,i),para.anchor_pos(2,i),100,'filled','MarkerFaceColor','black');
end
xlabel('x/mm')
ylabel('z/mm')
figtitle=['Robot Posture with stiffness=',num2str(para.leg_stiffness(1))];
title(figtitle,'FontSize',14)
legend([f1,f2,f3,f4],'Robot Body','virtual spring','physical leg','anchors','Location','northeast')

axis([-100,600,-300,400])

savefilename=['simulation_video/robot_posture_',num2str(frame_i),'.png'];

saveas(fig,savefilename,'png')
close(fig);