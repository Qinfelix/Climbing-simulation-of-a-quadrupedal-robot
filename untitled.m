clc
clear

load('potantial_energy_landscape_2_leganchor.mat','landscape')

landscape_plot=squeeze(landscape(1,:,:,1));

imagesc(landscape_plot')
axis xy
xlabel('x/mm')
ylabel('z/mm')
% axis off
colormap(fire)
colorbar

