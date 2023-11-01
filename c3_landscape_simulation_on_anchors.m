% Yue write this code at 2023/08/27 to run a simple simulation to see
% whether our climbing strategy based on potential energy landscape model
% is effective in the climbing process

clc 
clear

save_file_name='simulation_on_2_anchors_result.mat';
load('potantial_energy_landscape_2_leganchor.mat','landscape');
landscape_plot=squeeze(landscape(1,:,:,1));

is_plot=1;
is_draw_video=0;
is_save=0;

Tf=20;
dt=0.001;
frame_num=Tf/dt;

%%% parameter struct
para.mass=3;%kg
para.length=125;%mm
para.inertia=0.4;%kg*m*m (attention the inertial unit is m but not mm)
para.friction_k=[0.1,0.1];
para.leg_num=2;%number of legs when standing
%the index means different legs
para.leg_origin_pos(:,1)=[-80,180];%mm--coordinat:x_robot,y_robot
para.leg_stiffness(:,1)=[0.3,0.3];%N/mm
para.leg_damper(:,1)=[0.1,0.1];%N*s/mm
para.leg_standing_anchor(1)=1;
para.leg_connector_pos(:,1)=[-80,0];%means physical connection point

para.leg_origin_pos(:,2)=[80,180];%mm--coordinat:x_robot,y_robot
para.leg_stiffness(:,2)=[0.3,0.3];%N/mm
para.leg_damper(:,2)=[0.1,0.1];%N*s/mm
para.leg_standing_anchor(2)=2;
para.leg_connector_pos(:,2)=[80,0];

para.anchor_num=2;
%the index means different anchors same index with interacting leg
para.anchor_pos(:,1)=[150,10];%mm--coordinate:x,z
para.anchor_pos(:,2)=[350,60];%mm


%%% state_record_struct

all_states.frame_index=0;
all_states.t=nan(1,frame_num);
all_states.x=nan(1,frame_num);
all_states.z=nan(1,frame_num);
all_states.theta=nan(1,frame_num);
all_states.x_vel=nan(1,frame_num);
all_states.z_vel=nan(1,frame_num);
all_states.theta_vel=nan(1,frame_num);
all_states.x_acc=nan(1,frame_num);
all_states.z_acc=nan(1,frame_num);
all_states.theta_acc=nan(1,frame_num);
all_states.force_from_anchor=nan(para.anchor_num,2,frame_num);

% cache for state of t and t-1 
state_now.body.x=280;
state_now.body.z=100;
state_now.body.theta=0;%zero means x_robot is same as x;counter-clockwise is the positive direction
state_now.body.x_vel=0;
state_now.body.z_vel=0;
state_now.body.theta_vel=0;
state_now.t=0;
state_now.force_from_anchor=zeros(2,para.anchor_num);

state_old=state_now;

t_count=0;
%%% begin simulation
while (1)
    % check stop
    if state_now.t > Tf
        break;
    end
    %show the computing time
    if state_now.t >= t_count
        disp(state_now.t);
        t_count = t_count + 1;
    end

    % get previous
    state_old  = state_now;

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    force_from_anchor= f2_controller_rule(state_old,para);
    acc=f3_newton_sec_law(state_old,force_from_anchor,para);
    %iterate the state
    state_now.t=state_old.t+dt;
    state_now.body.x=state_old.body.x+state_old.body.x_vel*dt;
    state_now.body.z=state_old.body.z+state_old.body.z_vel*dt;
    state_now.body.theta=state_old.body.theta+state_old.body.theta_vel*dt;
    state_now.body.x_vel=state_old.body.x_vel+acc(1)*dt;
    state_now.body.z_vel=state_old.body.z_vel+acc(2)*dt;
    state_now.body.theta_vel=state_old.body.theta_vel+acc(3)*dt;
    state_now.force_from_anchor=force_from_anchor;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %record the state into memory
    all_states.frame_index = all_states.frame_index + 1;
    frame_i=all_states.frame_index;
    all_states.t(frame_i)=state_now.t;
    all_states.x(frame_i)=state_now.body.x;
    all_states.z(frame_i)=state_now.body.z;
    all_states.theta(frame_i)=state_now.body.theta;
    all_states.x_vel(frame_i)=state_now.body.x_vel;
    all_states.z_vel(frame_i)=state_now.body.z_vel;
    all_states.theta_vel(frame_i)=state_now.body.theta_vel;
    all_states.x_acc(frame_i)=acc(1);
    all_states.z_acc(frame_i)=acc(2);
    all_states.theta_acc(frame_i)=acc(3);

    all_states.force_from_anchor(:,:,frame_i)=state_now.force_from_anchor;

     %draw a picture of robot posture and anchors
     if is_draw_video
         if mod(frame_i,100)==0
             image_i=floor(frame_i/100);
             f4_plot_robot_posture(state_now,image_i,para);
         end
     end

end

if is_save
    save(save_file_name,'para','all_states');
end


%% plot


if is_plot
    %%% settings
    color.x = [1,0,0];
    color.z = [0,1,0];
    color.theta = [0,0,1];

    linewidth.thin = 1;
    linewidth.common = 2;

    %%% plots
    fig=figure;
    titlesize=20;
    set(fig,'Position',[100,100,1200,1200])
    subplot(3,3,1);
    plot(all_states.t, all_states.x,'Color',color.x,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('x/mm')
    title('x','FontSize',titlesize)
    subplot(3,3,2);
    plot(all_states.t, all_states.z,'Color',color.z,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('z/mm')
    title('z','FontSize',titlesize)
    subplot(3,3,3);
    plot(all_states.t, all_states.theta*180/pi,'Color',color.theta,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('theta/degree')
    title('theta','FontSize',titlesize)

    subplot(3,3,4);
    plot(all_states.t, all_states.x_vel,'Color',color.x,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('x vel(mm/s)')
    title('x vel','FontSize',titlesize)
    subplot(3,3,5);
    plot(all_states.t, all_states.z_vel,'Color',color.z,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('z vel(mm/s)')
    title('z vel','FontSize',titlesize)
    subplot(3,3,6);
    plot(all_states.t, all_states.theta_vel*180/pi,'Color',color.theta,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('theta vel(degree/s)')
    title('theta vel','FontSize',titlesize)

    subplot(3,3,7);
    plot(all_states.t, all_states.x_acc,'Color',color.x,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('x acc(mm/s2)')
    title('x acc','FontSize',titlesize)
    subplot(3,3,8);
    plot(all_states.t, all_states.z_acc,'Color',color.z,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('z acc(mm/s2)')
    title('z acc','FontSize',titlesize)
    subplot(3,3,9);
    plot(all_states.t, all_states.theta_acc*180/pi,'Color',color.theta,'LineWidth',linewidth.common)
    xlabel('t/s')
    ylabel('theta acc(degree/s2)')
    title('theta acc','FontSize',titlesize)

    saveas(fig,'state_curve.jpg','jpg')

end
