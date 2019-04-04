% Author : Anindya Harchowdhury
% Date : 11.01.2018
% Second Last modified : 31.01.2018
% Last modified : 14.05.2018
% Latest Modified : 03.02.2019
% NLP optimization to find out the optimal value of $\alpha$, $\beta$ and
% $d$ for calibrating the sensor while the mirrors are uplifted
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% lsqnonlin
% hold off; clear; close all
% clear laser_pitch x_loc y_loc z_loc X Y Z;
% clear; close all
% clear; close all;
% pitch_off = 3.4;


pitch_off = 0.0;
min_ang = -2.08007788658;
theta_res = 0.00613592332229;
val = 3652;
filename = 'scan_s1.txt';
matR = csvread(filename, 1, 11);
laser_time_inc = 97656.2514552 ;  % time resolution

T1 = readtable('scan_s1.txt', 'ReadVariableNames', false);
start = 2;
for i= start : 657

    a = T1{i,3};
%     b= cellstr(a);
%     c = char(b);
   e =  str2double(a);
    f1(1,i-1) = e;    % laser stamp
end

% % % for XM430_W210 robot


T2 = readtable('ser_s1.txt', 'ReadVariableNames', false);
for i= start : val
    a1(1,i-1) = T2{i,3};
    e1 = str2double(a1(1,i-1));
    f2(1,i-1) = e1;     % servo stamp
end

for i= start : val
    a2(1,i-1) = T2{i,13};
    e1 = str2double(a2(1,i-1));
    f3(1,i-1) = e1;     % servo position
end

T4 = readtable('vicon_s1.txt', 'ReadVariableNames', false);  % laptop pose

len =12672;
for i= start : len
    a4(1,i-1) = T4{i,3};
    e4 = str2double(a4(1,i-1));
    f4(1,i-1) = e4;     % robot stamp
end

for i= start : len
    a5(1,i-1) = T4{i,6};
    e5 = str2double(a5(1,i-1));
    f5(1,i-1) = e5;     % robot position -- x axis
end

for i= start : len
    a6(1,i-1) = T4{i,7};
    e6 = str2double(a6(1,i-1));
    f6(1,i-1) = e6;     % robot position -- y axis
end

for i= start : len
    a7(1,i-1) = T4{i,8};
    e7 = str2double(a7(1,i-1));
    f7(1,i-1) = e7;     % robot position -- z axis
end


filename = 'ser_s1.txt';
matA = csvread(filename, 1, 12);
matB = matA(:, 1) * 0.088 * pi/180;
length=1:size(matB);
[m, n]=size(matA);
% % len=601;
frame_num = 656;
laser_time_inc = 97656.2514552;  % time resolution
for l=2:frame_num-1
    laser_time(l,1:678) =  f1(l) + laser_time_inc*(0:677);
end
% Z0=0.1191;  % sensor height offset
Z0=0.1125;
% Servo pitch interpolation
theta_var = 0.0;
% theta_var = 0;
pitch_correction = 0.015;

% laser stamp, servo stamp and robot pose synchronisation using VICON data
count = 1;
for l=2:654
    if l==340
        aa=1;
    end
    i=1; c=1;
    while i~= 678
        if count <=len
            if count>1 && f4(count) - f4(count-1)>90000000
                lll = 3;
            end
            if laser_time(l,i) >= f4(count) && laser_time(l,i) <= f4(count+1) && f4(count+1) - f4(count)<90000000
                i1 = i;
                for j = 1: ceil((f4(count+1) - laser_time(l,i1))/laser_time_inc)
                    robo_x(l,i) =  f5(count) + (laser_time(l,i) - f4(count))* (f5(count+1) - f5(count))/ (f4(count+1) - f4(count));
                    robo_y(l,i) =  f6(count) + (laser_time(l,i) - f4(count))* (f6(count+1) - f6(count))/ (f4(count+1) - f4(count));
                    robo_z(l,i) =  f7(count) + (laser_time(l,i) - f4(count))* (f7(count+1) - f7(count))/ (f4(count+1) - f4(count));
                    
                    servo_x(l,i) = robo_x(l,i) + laptop_to_servo_x;
                    servo_y(l,i) = robo_y(l,i) + laptop_to_servo_y;
                    servo_z(l,i) = robo_z(l,i) + laptop_to_servo_z;
                    i = i + 1;
                    if i==678
                        break;
                    end
                end
                count = count + 1;
            elseif count >1 && count <=len && laser_time(l,i) >= f4(count-1) && laser_time(l,i) <= f4(count) && f4(count) - f4(count-1)<90000000
                i2 = i;
                for j = 1: ceil((f4(count) - laser_time(l,i2))/laser_time_inc)
                    i2 = i;
                    robo_x(l,i) = f5(count-1) +  (laser_time(l,i) - f4(count))* (f5(count) - f5(count-1))/ (f4(count+1) - f4(count));
                    robo_y(l,i) = f6(count-1) +  (laser_time(l,i) - f4(count))* (f6(count) - f6(count-1))/ (f4(count+1) - f4(count));
                    robo_z(l,i) = f7(count-1) +  (laser_time(l,i) - f4(count))* (f7(count) - f7(count-1))/ (f4(count+1) - f4(count));
                    
                    servo_x(l,i) = robo_x(l,i) + laptop_to_servo_x;
                    servo_y(l,i) = robo_y(l,i) + laptop_to_servo_y;
                    servo_z(l,i) = robo_z(l,i) + laptop_to_servo_z;
                    i = i + 1;
                    if i==678
                        break;
                    end
                end
                count = count + 1;                
            elseif count >1 && count<=len && laser_time(l,i) < f4(count) && laser_time(l,i) < f4(count-1)
                i3 = i;
                if i3>1
                    for j = 1: ceil((f4(count-2) - laser_time(l,i3))/laser_time_inc)
                        robo_x(l,i) = robo_x(l,i3-1) +  (laser_time(l,i) - laser_time(l,i3 - 1))* (f5(count-1) - robo_x(l,i3-1))/ (-laser_time(l,i3 - 1) + f4(count-1));
                        robo_y(l,i) = robo_y(l,i3-1) +  (laser_time(l,i) - laser_time(l,i3 - 1))* (f6(count-1) - robo_y(l,i3-1))/ (-laser_time(l,i3 - 1) + f4(count-1));
                        robo_z(l,i) = robo_z(l,i3-1) +  (laser_time(l,i) - laser_time(l,i3 - 1))* (f7(count-1) - robo_z(l,i3-1))/ (-laser_time(l,i3 - 1) + f4(count-1));
                        
                        servo_x(l,i) = robo_x(l,i) + laptop_to_servo_x;
                        servo_y(l,i) = robo_y(l,i) + laptop_to_servo_y;
                        servo_z(l,i) = robo_z(l,i) + laptop_to_servo_z;
                        i = i + 1;
                        if i==678
                            break;
                        end
                    end
                else
                    for j = 1: ceil((f4(count-2) - laser_time(l,i3))/laser_time_inc)
                        robo_x(l,i) = robo_x(l-1,677) +  (laser_time(l,i) - laser_time(l-1,677))* (f5(count-1) - robo_x(l-1,677))/ (-laser_time(l-1,677) + f4(count-1));
                        robo_y(l,i) = robo_y(l-1,677) +  (laser_time(l,i) - laser_time(l-1,677))* (f6(count-1) - robo_y(l-1,677))/ (-laser_time(l-1,677) + f4(count-1));
                        robo_z(l,i) = robo_z(l-1,677) +  (laser_time(l,i) - laser_time(l-1,677))* (f7(count-1) - robo_z(l-1,677))/ (-laser_time(l-1,677) + f4(count-1));
                        
                        servo_x(l,i) = robo_x(l,i) + laptop_to_servo_x;
                        servo_y(l,i) = robo_y(l,i) + laptop_to_servo_y;
                        servo_z(l,i) = robo_z(l,i) + laptop_to_servo_z;
                        i = i + 1;
                        if i==678
                            break;
                        end
                    end
                end
                count = count + 1;
            elseif count >1 && count<=len && laser_time(l,i) < f4(count) && laser_time(l,i) > f4(count-1)
                i4 = i;
                for j = 1: ceil((- f4(count-1) + laser_time(l,i4))/laser_time_inc)
                    robo_x(l,i) = f5(count-1) +  (laser_time(l,i) - f4(count-1))* (f5(count) - f5(count-1))/ (f4(count) - f4(count-1));
                    robo_y(l,i) = f6(count-1) +  (laser_time(l,i) - f4(count-1))* (f6(count) - f6(count-1))/ (f4(count) - f4(count-1));
                    robo_z(l,i) = f7(count-1) +  (laser_time(l,i) - f4(count-1))* (f7(count) - f7(count-1))/ (f4(count) - f4(count-1));
                    
                    servo_x(l,i) = robo_x(l,i) + laptop_to_servo_x;
                    servo_y(l,i) = robo_y(l,i) + laptop_to_servo_y;
                    servo_z(l,i) = robo_z(l,i) + laptop_to_servo_z;
                    i = i + 1;
                    if i==678
                        break;
                    end
                end
                count = count + 1;
            else
                % i = i +1;
                count = count + 1;
            end
        else
            break;
        end
    end
end


count = 1;

for l=2:654
    i=1; c=1;
    while i~= 678
        if count <=val && laser_time(l,i) >= f2(count) && laser_time(l,i) <= f2(count+1)
            i1 = i;
            for j = 1: ceil((f2(count+1) - laser_time(l,i1))/laser_time_inc)
                if ~isinf(matR(l,i)) && ~isnan(matR(l,i)) && matR(l,i)<=4
                    x_loc(l,i) = matR(l,i) * cos(i * theta_res + min_ang - theta_var);
                    y_loc(l,i) = matR(l,i) * sin(i * theta_res + min_ang - theta_var);
                    z_loc(l,i) = Z0;
                    %                     if f3(count) == f3(count +1)+
                    if matB(count) > matB(count + 1) && matB(count) < 3.4803
                        %                         laser_pitch(l,i) = matB(count) - pitch_correction + (laser_time(l,i)-f2(count))* (matB(count+1) - matB(count))/ (f2(count+1) - f2(count));
                        laser_pitch(l,i) = matB(count) - pitch_correction + (laser_time(l,i)-f2(count))* (matB(count+1) - matB(count))/ (f2(count+1) - f2(count));
                    else
                        laser_pitch(l,i) = matB(count) - pitch_correction + (laser_time(l,i)-f2(count))* (matB(count+1) - matB(count))/ (f2(count+1) - f2(count));
                    end
                    %                     else
                    %                         laser_pitch(l,i) = matB(count) + f3(count) * (laser_time(l,i)-f2(count)) + 0.5 * (f3(count) - (f3(count +1))/ (f2(count+1)...
                    %                             - f2(count))) * (laser_time(l,i)-f2(count))^2;
                    %                     end
                    %                     TF2 = [cos(laser_pitch(l,i)-3.35) 0 -sin(laser_pitch(l,i)-3.35) 0; 0 1 0 0; sin(laser_pitch(l,i)-3.35) 0 cos(laser_pitch(l,i)-3.35) 0; 0 0 0 1];
                    %                     if laser_pitch(l,i) < pi
                    
                    %                         TF = [cos(laser_pitch(l,i) - pitch_off) 0 sin(laser_pitch(l,i) - pitch_off); 0 1 0 ; -sin(laser_pitch(l,i) - pitch_off) 0 -cos(laser_pitch(l,i) - pitch_off)];
                    
                    
                    TF = [-cos(laser_pitch(l,i) + pitch_off) 0 -sin(laser_pitch(l,i) + pitch_off); 0 1 0 ; sin(laser_pitch(l,i) + pitch_off) 0 -cos(laser_pitch(l,i) + pitch_off)];
                    %                     V =  inv(TF2)*inv(TF1) * [x_loc(l,i) y_loc(l,i) z_loc(l,i) 1]';
                    %                     else
                    %                                                TF = [-cos(laser_pitch(l,i) + pitch_off) 0 -sin(laser_pitch(l,i) + pitch_off); 0 1 0 ; -sin(laser_pitch(l,i) + pitch_off) 0 cos(laser_pitch(l,i) + pitch_off)];
                    %
                    %                     end
                    V =  TF * [x_loc(l,i) y_loc(l,i) z_loc(l,i)]';
                    %                     X(l,i) = V(1) + robot_pose_x(l,i) + 0.07;
                    %                     Y(l,i) = V(2) + robot_pose_y(l,i);
                    
                    X(l,i) = V(1) + 0.07 - servo_y(l,i);
                    Y(l,i) = V(2) - servo_x(l,i);
                    Z(l,i) = V(3) + 0.2555 + servo_z(l,i);
                    
%                     X(l,i) = V(1) + 0.07 ;
%                     Y(l,i) = V(2) ;
%                     Z(l,i) = V(3) + 0.2555 ;
                    i=i+1;
                    %                     c=c+1;
                    if i==678
                        break;
                    end
                else
                    i=i+1;
                    if i==678
                        break;
                    end
                end
            end
            count = count+1;
            t=1;
        elseif count >1 && count <=val && laser_time(l,i) >= f2(count-1) && laser_time(l,i) <= f2(count)
            i2 = i;
            for j = 1: ceil((f2(count) - laser_time(l,i2))/laser_time_inc)
                if ~isinf(matR(l,i)) && ~isnan(matR(l,i)) && matR(l,i)<=4
                    x_loc(l,i) = matR(l,i) * cos(i * theta_res + min_ang - theta_var);
                    y_loc(l,i) = matR(l,i) * sin(i * theta_res + min_ang - theta_var);
                    z_loc(l,i) = Z0;
                    %                     if f3(count) == f3(count -1)
                    if matB(count-1) > matB(count) && matB(count) < 3.4803
                        
                        %                         laser_pitch(l,i) = matB(count-1) - pitch_correction +  (laser_time(l,i) - f2(count))* (matB(count) - matB(count-1))/ (f2(count+1) - f2(count));
                        laser_pitch(l,i) = matB(count-1) - pitch_correction +  (laser_time(l,i) - f2(count))* (matB(count) - matB(count-1))/ (f2(count+1) - f2(count));
                    else
                        laser_pitch(l,i) = matB(count-1) - pitch_correction +  (laser_time(l,i) - f2(count))* (matB(count) - matB(count-1))/ (f2(count+1) - f2(count));
                    end
                    %                     else
                    %                         laser_pitch(l,i) = matB(count-1) + f3(count-1) * (laser_time(l,i)-f2(count-1)) + 0.5 * (f3(count) - (f3(count -1))/ (f2(coun)...
                    %                             - f2(count-1))) * (laser_time(l,i)-f2(count-1))^2;
                    %                     end
                    
                    %                     TF1 = [1 0 0 0;0 1 0 0; 0 0 1 z_loc(l,i); 0 0 0 1];
                    %                     TF2 = [-cos(laser_pitch(l,i)-3.35) 0 sin(laser_pitch(l,i)-3.35) 0; 0 1 0 0; -sin(laser_pitch(l,i)-3.35) 0 -cos(laser_pitch(l,i)-3.35) 0; 0 0 0 1];
                    %                     V =  inv(TF2)*inv(TF1) * [x_loc(l,i) y_loc(l,i) z_loc(l,i) 1]';
                    TF = [-cos(laser_pitch(l,i) - pitch_off) 0 -sin(laser_pitch(l,i) - pitch_off); 0 1 0 ; sin(laser_pitch(l,i) - pitch_off) 0 -cos(laser_pitch(l,i) - pitch_off)];
                    V =  TF * [x_loc(l,i) y_loc(l,i) z_loc(l,i)]';
                    %                     X(l,i) = V(1) + robot_pose_x(l,i) + 0.07;
                    %                     Y(l,i) = V(2) + robot_pose_y(l,i);
                    X(l,i) = V(1) + 0.07 - servo_y(l,i);
                    Y(l,i) = V(2) - servo_x(l,i);
                    Z(l,i) = V(3) + 0.2555 + servo_z(l,i);

%                     X(l,i) = V(1) + 0.07;
%                     Y(l,i) = V(2);
%                     Z(l,i) = V(3) + 0.2555 ;
                    i=i+1;
                    %                     c= c+1;
                    if i==678
                        break;
                    end
                else
                    i=i+1;
                    if i==678
                        break;
                    end
                end
            end
            count = count+1;
            t=1;
        elseif count ==1 && laser_time(l,i) < f2(count)
            count = count +1;
            t=1;
            
        elseif count >=1 && count <val && laser_time(l,i) > f2(count)
            count = count +1;
            t=1;
        else
            i=i+1;
            if i==678
                break;
            end
        end
        if i==678
            break;
        end
        
    end
end


% for i =1:499
%     B(i) = nnz(X(i,:));
% end
figure
for i=2:654
    scatter3(100*X(i,256:450), 100*Y(i,256:450), 100*Z(i,256:450),1, 'b','o');
    hold on
end
% 
% % figure
% % for i=1:700
% %     scatter3(100*X(i,1:677), 100*Y(i,1:677), 100*Z(i,1:677),1, 'b','o');
% %     hold on
% % end
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %  Plane fitting
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% c=1;
% for i=1:399
%     for j=226:450
%         if X(i,j)>1.51
%             M1(c,1)=X(i,j);
%             M1(c,2)=Y(i,j);
%             M1(c,3)=Z(i,j);
%             c=c+1;
%         end
%     end
% end
% 
% hold on
% [coeff,score,roots] = pca(M1);
% basis = coeff(:,1:2);
% 
% normal = coeff(:,3);
% 
% pctExplained = roots' ./ sum(roots);
% 
% [n,p] = size(M1);
% meanX = mean(M1,1);
% Xfit = repmat(meanX,n,1) + score(:,1:2)*coeff(:,1:2)';
% residuals = M1 - Xfit;
% 
% error = abs((M1 - repmat(meanX,n,1))*normal);
% sse = sum(error.^2);
% 
% [xgrid,ygrid] = meshgrid(linspace(min(M1(:,1)),max(M1(:,1)),5), ...
%     linspace(min(M1(:,2)),max(M1(:,2)),5));
% zgrid = (1/normal(3)) .* (meanX*normal - (xgrid.*normal(1) + ygrid.*normal(2)));
% h = mesh(xgrid,ygrid,zgrid,'EdgeColor',[0 0 0],'FaceAlpha',0);
% 
% hold on
% above = (M1-repmat(meanX,n,1))*normal < 0;
% below = ~above;
% nabove = sum(above);
% X1 = [M1(above,1) Xfit(above,1) nan*ones(nabove,1)];
% X2 = [M1(above,2) Xfit(above,2) nan*ones(nabove,1)];
% X3 = [M1(above,3) Xfit(above,3) nan*ones(nabove,1)];
% plot3(X1',X2',X3','-', M1(above,1),M1(above,2),M1(above,3),'o', 'MarkerSize', 1, 'Color',[0 .7 0]);
% nbelow = sum(below);
% X1 = [M1(below,1) Xfit(below,1) nan*ones(nbelow,1)];
% X2 = [M1(below,2) Xfit(below,2) nan*ones(nbelow,1)];
% X3 = [M1(below,3) Xfit(below,3) nan*ones(nbelow,1)];
% plot3(X1',X2',X3','-', M1(below,1),M1(below,2),M1(below,3),'o', 'MarkerSize', 1, 'Color',[1 0 0]);
% 
% hold off
% maxlim = max(abs(M1(:)))*1.1;
% axis([0 maxlim -maxlim maxlim -0.5 1.8]);
% axis square
% view(-9,12);
% Z_Loc = 0.1191;
for i = 1:682
    angle(i) = (i-1) * theta_res + min_ang;
end
% for l=2:300
%     if (laser_pitch(l,340)) <= 0.3 && (laser_pitch(l,340)) >= -0.3
%         range_zero(l-1) = -Z_Loc*sin(laser_pitch(l,340) - 3.4) - matR(l,340)*cos(laser_pitch(l,340) - 3.4)*cos(339 * theta_res + min_ang);
%     end
% end
% range_mean = sum(range_zero)/nnz(range_zero);
% 
% % cartesian coordinate of the point hit by the beam emanting from the front
% % of the sensor center, essentially from the perpendicular direction
% [x,y,z] = sph2cart(0, 0, range_mean);
% 
% ax = atan2(norm(cross(normal,[x;y;z])),dot(normal,[x;y;z]));  % angle between the normal and the perpendicular range vector
% 
% Z_off = 0.119;
% pitch_off = 3.4;
% mult = 1.0;
% L1_angle = 1.41; L2_angle = 1.0617;
% % L1_angle = 1.3970; L2_angle = 1.0617;
% R1_angle = 1.4; R2_angle = 1.0648;
% perp_1 = 0.02; perp_2 = 0.07;
% L1_distance = 0.0587; L2_distance = 0.08224;
% 
% aa=1;


% No computation from here until PITCH_COMP


% for l = 2:300
%     for k = 1:113
%         perp_dist(k) = (range_mean + Z_Loc*sin(laser_pitch(l,k) - 3.4)) / -cos(laser_pitch(l,k) - 3.4);
%         %         perp_dist = matR(l,k)* cos(k * theta_res + min_ang);
%         if k ==1
%             ABO(k) = R1_angle;
%             OC(k) = L1_distance * sin(pi - 2 * R1_angle);
%             XO(k) = OC(k) / cos(abs(angle(k)) - (-(pi/2) + 2 * R1_angle));
%             XC(k) = OC(k) * tan(abs(angle(k)) - (-(pi/2) + 2 * R1_angle));
%             XM(k) = perp_dist(k) - XO(k);
%             if ax <0
%                 X_pX(k) = XM(k) * cos(2 * R1_angle - abs(angle(k)));
%                 X_pM(k) = XM(k) * sin(2 * R1_angle - abs(angle(k)));
%                 D_p_X_p(k) = X_pM(k) * tan(pi -2 * R1_angle + ax);
%                 range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - ((pi/2) - (2 * R1_angle - abs(angle(k))));
%             elseif ax >0
%                 D_primeX(k) = XM(k) / cos(2 * R1_angle - abs(angle(k)));  % (pi - (pi -2 * R1_angle + angle(k)))
%                 D_primeM(k) = XM(k) * tan(2 * R1_angle - abs(angle(k)));
%                 D_p_X_p(k) = D_primeM(k) * sin(ax);
%                 D_primeD(k) = D_p_X_p(k) / sin((pi/2) + abs(ax) -(2 * R1_angle - abs(angle(k))));
%                 range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - ((pi/2) - (2 * R1_angle - abs(angle(k))));
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(abs(angle(k)));
%             y1_loc(l,k) = range_des(l,k)*sin(abs(angle(k)));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V = TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%             OA(k) = L1_distance;
%         else
%
%             if (pi - 2*ABO(k)) < pi/2
%
%                 OC(k) = OA(k) * sin(pi - 2 * ABO(k));
%                 Ang1(k) = abs(angle(k)) - ((pi/2) - OAC(k));
%                 Ang2(k) = 2 * ABO(k) - abs(angle(k)) + ax;
%
%                 XO(k) = OC(k) / cos(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%
%                     X_pX(k) = XM(k) * cos(2 * ABO(k) - abs(angle(k)));
%                     X_pM(k) = XM(k) * sin(2 * ABO(k) - abs(angle(k)));
%                     D_p_X_p(k) = X_pM(k) * tan(2 * ABO(k) - abs(angle(k)) + ax);
%                     Ang3(k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%                 elseif ax >0
%                     D_primeX(k) = XM(k) / cos(pi - (OAC(k) + abs(angle(k))));
%                     D_primeM(k) = XM(k) * tan(pi - (OAC(k) + abs(angle(k))));
%                     D_p_X_p(k) = D_primeM(k) * sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (pi - (OAC(k) + abs(angle(k)))) + ax);
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%                 end
%
%             elseif (pi - 2*ABO(k)) > pi/2
%                 OAC(k) = 2 * ABO(k);
%                 OC(k) = OA(k) * sin( 2 * ABO(k));
%                 XO(k) = OC(k) / cos(abs(angle(k)) + ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(abs(angle(k)) + ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     X_pX(k) = XM(k) * cos( OAC(k) - abs(angle(k)));
%                     X_pM(k) = XM(k) * sin(OAC(k) - abs(angle(k)));
%                     D_p_X_p(k) = X_pM(k) * tan(OAC(k) - abs(angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2);
%                     Ang3(k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                     angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%                 elseif ax >0
%                     D_primeX(k) = XM(k) / cos(OAC(k) - abs(angle(k)));
%                     D_primeM(k) = XM(k) * tan(OAC(k) - abs(angle(k)));
%                     D_p_X_p(k) = D_primeM(k) *sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin(OAC(k) - abs(angle(k)) + abs(ax));
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%                 end
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%         end
%         ABO(k+1) = ABO(k) - theta_res;
%         ON(k+1) = OA(k) * cos(theta_res);
%         AN(k+1) = OA(k) * sin(theta_res);
%         OAC(k+1) = pi - 2 * ABO(k+1) ;
%         NB(k+1) = AN(k+1) / tan(ABO(k+1));
%         OA(k+1) = (AN(k+1) + ON(k+1));
%     end
%     for k = 114:226
%         perp_dist(k) = (range_mean + Z_Loc*sin(laser_pitch(l,k) - 3.4)) / -cos(laser_pitch(l,k) - 3.4);
%         if k==114
%             ABO(k) = R2_angle;
%             OAC(k) = pi -2 * R2_angle;
%             OC(k) = L2_distance * sin(OAC(k));
%             XO(k) = OC(k) / cos(abs(angle(k)) + ((pi/2) - OAC(k)));
%             XC(k) = OC(k) * tan(abs(angle(k)) + (pi/2) - OAC(k));
%             XM(k) = perp_dist(k) - XO(k);
%
%             if ax < 0
%                 X_pX(k) = XM(k) * cos(OAC(k) - abs(angle(k)));
%                 X_pM(k) = XM(k) * sin(OAC(k) - abs(angle(k)));
%                 D_p_X_p(k) = X_pM(k) * tan(OAC(k) - abs(angle(k)) + abs(ax));
%                 range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%             elseif ax > 0
%                 D_primeX(k) = XM(k) / cos(OAC(k) - abs(angle(k)));
%                 D_primeM(k) = XM(k) * tan(OAC(k) - abs(angle(k)));
%                 D_p_X_p(k) = D_primeM(k) * sin(ax);
%                 D_primeD(k) = D_p_X_p(k) / sin((pi/2) -( OAC(k) - abs(angle(k))) + ax);
%                 range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%
%             OA(k) = L2_distance;
%         else
%             %             ABO = pi - (R2_angle + (k - 114)* theta_res);
%
%             if (pi - 2*ABO(k)) < pi/2
%
%                 OAC(k) = pi - 2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     X_pX(k) = XM(k) * cos(pi -( OAC(k) + abs(angle(k))));
%                     X_pM(k) = XM(k) * sin(pi -( OAC(k) + abs(angle(k))));
%                     D_p_X_p(k) = X_pM(k) * tan(pi -( OAC(k) + abs(angle(k))) + abs(ax));
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                     angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%                 elseif ax >0
%                     D_primeX(k) = XM(k) / cos(pi -(OAC(k) + abs(angle(k))));
%                     D_primeM(k) = XM(k) * tan(pi -(OAC(k) + abs(angle(k))));
%                     D_p_X_p(k) = D_primeM(k) * sin(ax);
%                     D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (pi - (OAC(k) + abs(angle(k)))) + ax);
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%                 end
%             elseif (pi - 2*ABO(k)) > pi/2
%
%                 OAC(k) = 2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(abs(angle(k)) + ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(abs(angle(k)) + ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     X_pX(k) = XM(k) * cos(OAC(k) - abs(angle(k)));
%                     X_pM(k) = XM(k) * sin(OAC(k) - abs(angle(k)));
%                     D_p_X_p(k) = X_pM(k) * tan(OAC(k) - abs(angle(k)) + abs(ax));
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                     angle_des(l,k) = atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%                 elseif ax >0
%                     D_primeX(k) = XM(k) / cos(OAC(k) - abs(angle(k)));
%                     D_primeM(k) = XM(k) * tan(OAC(k) - abs(angle(k)));
%                     D_p_X_p(k) = D_primeM(k) *sin(ax);
%                     D_primeD(k) = D_p_X_p(k) / sin((OAC(k) - abs(angle(k))) + ax);
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - ((pi/2) - OAC(k) + abs(angle(k)));
%                 end
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%         end
%         ABO(k+1) = ABO(k) - theta_res;
%         ON(k+1) = OA(k) * cos(theta_res);
%         AN(k+1) = OA(k) * sin(theta_res);
%         OAC(k+1) = pi - 2 * ABO(k+1) ;
%         NB(k+1) = AN(k+1) / tan(ABO(k+1));
%         OA(k+1) = (AN(k+1) + ON(k+1));
%     end
%     for k = 569:-1:457
%         perp_dist(k) = (range_mean + Z_Loc*sin(laser_pitch(l,k) - 3.4)) / -cos(laser_pitch(l,k) - 3.4);
%         if k == 569
%             OAC(k) = pi - 2 * L2_angle;
%             OC(k) = L2_distance * sin(OAC(k));
%             XO(k) = OC(k) / cos(angle(k) - ((pi/2) - OAC(k)));
%             XC(k) = OC(k) * tan(angle(k) - ((pi/2) - OAC(k)));
%             XM(k) = perp_dist(k) - XO(k);
%             if ax <0
%                 D_primeX(k) = XM(k) / cos(pi - (OAC(k) + angle(k)));
%                 D_primeM(k) = XM(k) * tan(pi - (OAC(k) + angle(k)));
%                 D_p_X_p(k) = D_primeM(k) * sin(ax);
%                 D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (OAC(k) - angle(k)) + abs(ax));
%                 range_des(l,k) = sqrt((D_primeX - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = ((pi/2) - OAC(k) + abs(angle(k))) - atan((D_primeX - D_primeD(k) + XC(k)) / OC(k));
%
%             elseif ax >0
%                 X_pX(k) = XM(k) * cos(pi - (OAC(k) + angle(k)));
%                 X_pM(k) = XM(k) * sin(pi - (OAC(k) + angle(k)));
%                 D_p_X_p(k) = X_pM(k) * tan(pi - (OAC(k) + angle(k)) + ax);
%                 range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                 angle_des(l,k) = (abs(angle(k)) - ((pi/2) - OAC(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0
%             -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%             ABO(k) = L2_angle;
%             OA(k) = L2_distance;
%         else
%
%             if (pi - 2*ABO(k)) < pi/2
%                 OAC(k) = pi -2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(abs(angle(k)) - ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     D_primeX(k) = XM(k) / cos(pi - (OAC(k)+ abs(angle(k))));
%                     D_primeM(k) = XM(k) * tan(pi - (OAC(k) + abs(angle(k))));
%                     D_p_X_p(k) = D_primeM(k) * sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (OAC(k) + angle(k)) + abs(ax));
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = (abs(angle(k)) - ((pi/2) - OAC(k))) - atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k));
%                 elseif ax >0
%                     X_pX(k) = XM(k) * cos(pi - (OAC(k) + angle(k)));
%                     X_pM(k) = XM(k) * sin(pi - (OAC(k) + angle(k)));
%                     D_p_X_p(k) = X_pM(k) * tan(pi - (OAC(k) + angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                     angle_des(l,k) = (abs(angle(k)) - ((pi/2) - OAC(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                 end
%             elseif (pi - 2*ABO(k)) > pi/2
%
%                 OAC(k) = 2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(angle(k) + ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(angle(k) + ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 Ang4(k) = OAC(k) - angle(k);
%                 if ax <0
%                     D_primeX(k) = XM(k) / cos(OAC(k) - abs(angle(k)));
%                     D_primeM(k) = XM(k) * tan(OAC(k) - abs(angle(k)));
%                     D_p_X_p(k) = D_primeM(k) *sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (OAC(k) - abs(angle(k))) + abs(ax));
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = ((pi/2) - OAC(k) + abs(angle(k))) - atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k));
%                 elseif ax >0
%                     X_pX(k) = XM(k) * cos(OAC(k) - angle(k));
%                     X_pM(k) = XM(k) * sin(OAC(k) - angle(k));
%                     D_p_X_p(k) = X_pM(k) * tan(OAC(k) - abs(angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                     angle_des(l,k) = ((pi/2) - OAC(k) + abs(angle(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                 end
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%         end
%         ABO(k-1) = ABO(k) - theta_res;
%         ON(k-1) = OA(k) * cos(theta_res);
%         AN(k-1) = OA(k) * sin(theta_res);
%         OAC(k-1) = pi - 2 * ABO(k-1);
%         NB(k-1) = AN(k-1) / tan(ABO(k-1));
%         OA(k-1) = (AN(k-1) + ON(k-1));
%     end
%     OA(682) = L1_distance; ABO(682) = L1_angle;
%     for k = 682:-1:570
%         ABO(k-1) = ABO(k) - theta_res;
%         ON(k-1) = OA(k) * cos(theta_res);
%         AN(k-1) = OA(k) * sin(theta_res);
%         OAC(k-1) = pi - 2 * ABO(k-1);
%         NB(k-1) = AN(k-1) / tan(ABO(k-1));
%         OA(k-1) = (AN(k-1) + ON(k-1));
%     end
%     for k = 677:-1:570
%         perp_dist(k) = (range_mean + Z_Loc*sin(laser_pitch(l,k) - 3.4)) / -cos(laser_pitch(l,k) - 3.4);
%         if k == 682
%             ABO(k) = L1_angle;
%             OA(k) = L1_distance;
%             OAC(k) = pi -2 * L1_angle;
%             OC(k) = L1_distance * sin(OAC(k));
%             XO(k) = OC(k) / cos(angle(k) -((pi/2) - OAC(k)));
%             XC(k) = OC(k) * tan(angle(k) -((pi/2) - OAC(k)));
%             XM(k) = perp_dist(k) - XO(k);
%             if ax <0
%                 D_primeX(k) = XM(k) / cos(pi - (OAC(k) + angle(k)));
%                 D_primeM(k) = XM(k) * tan(pi - (OAC(k) + angle(k)));
%                 D_p_X_p(k) = D_primeM(k) * sin(abs(ax));
%                 D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (pi - (OAC(k) + abs(angle(k)))) + ax);
%                 range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                 angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%             elseif ax >0
%                 X_pX(k) = XM(k) * cos(pi - (OAC(k) + angle(k)));
%                 X_pM(k) = XM(k) * sin(pi - (OAC(k) + angle(k)));
%                 D_p_X_p(k) = X_pM(k) * tan(pi - (OAC(k) + angle(k)) + ax);
%                 range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                 angle_des(l,k) = (abs(angle(k)) - ((pi/2) - OAC(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%             OA(k) = L1_distance;
%
%         else
%
%             if (pi - 2*ABO(k)) < pi/2
%                 OAC(k) = pi -2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(angle(k) - ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(angle(k) - ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     D_primeX(k) = XM(k) / cos(pi - (OAC(k) + angle(k)));
%                     D_primeM(k) = XM(k) * tan(pi - (OAC(k) + angle(k)));
%                     D_p_X_p(k) = D_primeM(k) * sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin((pi/2) - (OAC(k) + angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k)) - (abs(angle(k)) - ((pi/2) - OAC(k)));
%                 elseif ax >0
%                     X_pX(k) = XM(k) * cos(pi - (OAC(k) + angle(k)));
%                     X_pM(k) = XM(k) * sin(pi - (OAC(k) + angle(k)));
%                     D_p_X_p(k) = X_pM(k) * tan(pi - (OAC(k) + angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = (abs(angle(k)) - ((pi/2) - OAC(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                 end
%
%             elseif (pi - 2*ABO(k)) > pi/2
%                 OAC(k) = 2 * ABO(k);
%                 OC(k) = OA(k) * sin(OAC(k));
%                 XO(k) = OC(k) / cos(angle(k) + ((pi/2) - OAC(k)));
%                 XC(k) = OC(k) * tan(angle(k) + ((pi/2) - OAC(k)));
%                 XM(k) = perp_dist(k) - XO(k);
%                 if ax <0
%                     D_primeX(k) = XM(k) / cos(OAC(k) - angle(k));
%                     D_primeM(k) = XM(k) * tan(OAC(k) - angle(k));
%                     D_p_X_p(k) = D_primeM(k) *sin(abs(ax));
%                     D_primeD(k) = D_p_X_p(k) / sin(pi-((pi/2) - (OAC(k) - abs(angle(k))) + abs(ax)));
%                     range_des(l,k) = sqrt((D_primeX(k) - D_primeD(k) + XC(k))^2 + OC(k)^2);
%                     angle_des(l,k) = ((pi/2) - OAC(k) + abs(angle(k))) - atan((D_primeX(k) - D_primeD(k) + XC(k)) / OC(k));
%
%                 elseif ax >0
%                     X_pX(k) = XM(k) * cos(OAC(k) - angle(k));
%                     X_pM(k) = XM(k) * sin(OAC(k) - angle(k));
%                     D_p_X_p(k) = X_pM(k) * tan(OAC(k) - abs(angle(k)) + ax);
%                     range_des(l,k) = sqrt((D_p_X_p(k) + X_pX(k) + XC(k))^2 + OC(k)^2) ;
%                     angle_des(l,k) = ((pi/2) - OAC(k) + abs(angle(k))) - atan((D_p_X_p(k) + X_pX(k) + XC(k)) / OC(k));
%                 end
%             end
%             x1_loc(l,k) = range_des(l,k)*cos(angle_des(l,k));
%             y1_loc(l,k) = range_des(l,k)*sin(angle_des(l,k));
%             TF = [-cos(laser_pitch(l,k)-3.4) 0 -sin(laser_pitch(l,k)-3.4); 0 1 0 ; sin(laser_pitch(l,k)-3.4) 0 -cos(laser_pitch(l,k)-3.4)];
%             V =  TF * [x1_loc(l,k) y1_loc(l,k) z_loc(l,k)]';
%             X1(l,k) = V(1);
%             Y1(l,k) = V(2);
%             Z1(l,k) = V(3);
%         end
%         %         ABO(k-1) = ABO(k) - theta_res;
%         %         ON(k-1) = OA(k) * cos(theta_res);
%         %         AN(k-1) = OA(k) * sin(theta_res);
%         %         OAC(k-1) = pi - 2 * ABO(k-1);
%         %         NB(k-1) = AN(k-1) / tan(ABO(k-1));
%         %         OA(k-1) = (AN(k-1) + ON(k-1));
%     end
% end

% parameter estimation
% design variables
global alpha perp height;
alpha = 2.2552;
perp = 0.0579;
height = 0.02; % height in meter.

global range pitch_comp theta_comp;

for l = 1:900
    range((l-1) * 113 + 1: l * 113) = matR(l,1:113);
    pitch_comp((l-1) * 113 + 1: l * 113) = laser_pitch(l,1:113);
    theta_comp((l-1) * 113 + 1: l * 113) = angle(1:113);
end

for l = 1:900
    range2((l-1) * 113 + 1: l * 113) = matR(l,114:226);
    pitch_comp2((l-1) * 113 + 1: l * 113) = laser_pitch(l,114:226);
    theta_comp2((l-1) * 113 + 1: l * 113) = angle(114:226);
end

for l = 1:900
    range3((l-1) * 113 + 1: l * 113) = matR(l,457:569);
    pitch_comp3((l-1) * 113 + 1: l * 113) = laser_pitch(l,457:569);
    theta_comp3((l-1) * 113 + 1: l * 113) = angle(457:569);
end

for l = 1:900
    range4((l-1) * 107 + 1: l * 107) = matR(l,570:676);
    pitch_comp4((l-1) * 107 + 1: l * 107) = laser_pitch(l,570:676);
    theta_comp4((l-1) * 107 + 1: l * 107) = angle(570:676);
end


% domain of variables

del_alpha0 = 0.05; del_beta0 = 0.02; del_d0 = 0.003;     % initial value
param0 = [del_alpha0, del_beta0, del_d0];
lb = [-0.1, -0.05, -0.004];
ub = [0.1, 0.05, 0.004];
% objective function
% for l = 1:404 * 113
r_i_p(l) = range(l) - ((h * tan(param(2)) + d + param(3)) * tan(alpha + param(1) -theta_comp(l))) * cos(2 * param(2));

r_i_a(l) = sqrt((r_i_p(l) - (h * tan(param(2)) + d + param(3)) * cos(2 * param(2)))^2 + ((h * tan(param(2))...
    + d + param(3)) * tan(alpha + param(1) - theta_comp(l)))^2);

theta_i_a(l) = atan((r_i_p(l) - (h * tan(param(2)) + d + param(3))) / ((h * tan(param(2)) + d + param(3)) * tan(alpha + param(1) - ...
    theta_comp(l)))) - theta_comp(l) -2 * L1_angle + 2 * theta_res * mod((l-1),113) + pi;

x_loc(l) = r_i_a(l) * sin(theta_i_a(l));

y_loc(l) = r_i_a(l) * cos(theta_i_a(l));

z_loc(l) = r_i_p(l) * cos(2 * param(2));

TF = [-cos(pitch_comp(l) - 3.4) 0 -sin(pitch_comp(l) - 3.4); 0 1 0 ; sin(pitch_comp(l) - 3.4) 0 -cos(pitch_comp(l) - 3.4)];

V =  TF * [x_loc(l) y_loc(l) z_loc(l)]';
%
%     sqd_distance(l) = (dot((TF * V) , normal)/norm(normal))^2;
% end

% options = optimoptions(@lsqnonlin,'Algorithm','trust-region-reflective');
% options.Algorithm = 'levenberg-marquardt';
options = optimoptions('lsqnonlin','Display','iter');
[param,resnorm,residual,exitflag,output] = lsqnonlin(@myfun, param0, lb, ub, options, range, pitch_comp, theta_comp, normal);
% [x,resnorm,residual,exitflag,output] = lsqnonlin(fun,x0,[],[],options);
