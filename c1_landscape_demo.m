% Yue write this code to build a landscape modelling demo on Aug.16

%% calcute the energy landscape
close all; clear; clc;

is_save=0;
is_plot=1;

%grid is a struct of the coordinate infomation
grid.var(1).meaning='x';
grid.var(1).unit=1;
grid.var(1).lim=[-100,500];
grid.var(2).meaning='z';
grid.var(2).unit=1;
grid.var(2).lim=[0,300];
grid.var(3).meaning='theta';
grid.var(3).unit=pi;
grid.var(3).lim=[0,2*pi];
for i = 1:3
    grid.var(i).length=(grid.var(i).lim(2)-grid.var(i).lim(1))/grid.var(i).unit + 1;
    grid.var(i).range=grid.var(i).lim(1):grid.var(i).unit:grid.var(i).lim(2);
end

% record place of the landscape value
landscape=nan(2,grid.var(1).length,grid.var(2).length,grid.var(3).length);
%tempory variable representing the position index in the landscape grid
index=zeros(1,3);
%tempory variable representing the position real value now -- not same as
%the index
input=zeros(1,3);


setting.leg_num=2;
setting.situation_num=2;
setting.pos_set(:,:,1)=[150,10;350,60];
setting.pos_set(:,:,2)=[350,60;350,60];
setting.spring_para.origin_pos=[0,-180];
setting.spring_para.stiffness=[0.3,0.3];
setting.robot_mass=3;

for pos_set_i=1:2
    anchor_pos=setting.pos_set(:,:,pos_set_i);
    index(1)=0;
    %x
    for v1 = grid.var(1).range
        index(1) = index(1)+1;
        input(1) = v1;
        index(2) = 0;
        %display the computing process
        if mod(index(1),10)==0
            display(floor(index(1)/10))
        end
        % z
        for v2 = grid.var(2).range
            index(2) = index(2)+1;
            input(2) = v2;
            index(3) = 0;
             %%% theta
            for v3 = grid.var(3).range
                index(3) = index(3)+1;
                input(3) = v3;
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                %calculate the potential energy
                landscape(pos_set_i,index(1),index(2),index(3))=f1_calculate_potential_energy(anchor_pos,input,setting);
                %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
            end
        end
    end
end
if is_save
    save('potantial_energy_landscape_2_leganchor.mat','landscape');
end

%% plot the energy landscape

if is_plot
    theta_index=1;
    z_index=2;
    
    landscape_plot_1=squeeze(landscape(1,:,:,theta_index));
%     landscape_plot_2=squeeze(landscape(2,:,_index,:));
    
    %compute the landscape of minimum value of all the theta/pos_set situation
    %landscape_plot_min=min(landscape,[],3);
    %landscape_plot=min(landscape_plot_2,landscape_plot_1);
    
%     [X,THETA]=meshgrid(grid.var(1).range,grid.var(3).range*180/pi);
%     surf(THETA',X',landscape_plot_1,'EdgeColor','none')
%     xlabel('theta/degree')
    [X,Z]=meshgrid(grid.var(1).range,grid.var(2).range);
    surf(Z',X',landscape_plot_1,'EdgeColor','none')
    xlabel('z/mm')
    
    colormap(fire);

    

    % % plot the landscape of 2 under position sets

    % surf(X',Z',landscape_plot_1,'EdgeColor','none');
    % hold on 
    % surf(X',Z',landscape_plot_2,'EdgeColor','none');
    
    hold on 

    
    title('potential energy landscape')
    
    ylabel('x/cm')

    zlabel('potential energy (J)')
    
    
    %plot the anchor position
    % for i=1:3
    %     plot3(anchor_pos_all(i,1,1),anchor_pos_all(i,2,1),0,'.r','markersize',20)
    % end
    % 
    % for j=1:3
    %     plot3(anchor_pos_all(4-j,1,2),anchor_pos_all(4-j,2,2),0,'b-*','markersize',20)
    % end
    % % legend('theta = 40 degree with red anchor','theta = 40 degree with blue anchor','Location','best')
    % legend('theta = 40 degree ','Location','best')
end


