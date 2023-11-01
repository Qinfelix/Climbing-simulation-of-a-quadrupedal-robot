function [acc] = f3_newton_sec_law(state,force_from_anchor,para)
%F3_NEWTON_SEC_LAW Summary of this function goes here
%   Detailed explanation goes here
acc=zeros(3,1);

for anchor_i=1:para.anchor_num
    fx=force_from_anchor(1,anchor_i);
    fz=force_from_anchor(2,anchor_i);
    acc(1)=acc(1)+fx/para.mass;
    acc(2)=acc(2)+fz/para.mass;
    moment_fx=(state.body.z-para.anchor_pos(2,anchor_i))*fx;
    moment_fz=(para.anchor_pos(1,anchor_i)-state.body.x)*fz;
    acc(3)=acc(3)+(moment_fx+moment_fz)/1000/para.inertia;
end
acc(2)=acc(2)-9.8;

%give a energy loss in theta dimension
moment_friction_acc=1;
if abs(acc(3))<moment_friction_acc
    acc(3)=0;
else
    if acc(3)>0
        acc(3)=acc(3)-moment_friction_acc;
    else
        acc(3)=acc(3)+moment_friction_acc;
    end
end


end

