% % Author : Anindya Harchowdhury
% % Description: dynamic obstacle detection and its state estimation
% % Started: 26.12.2018
% % second last Updated: 27.02.2019
% % last updated: 07.03.2019
% % environment details
% % 4m in both x, y ,z direction
%
%
% x_off = 4; x_off = 4; x_off =4;
%
% X_new = X + x_off;
% Y_new = Y + y_off;
% Z_new = Z + z_off;
% % for i=1:400
% % end

% Modification due to mirror set-up

% for i=2:899
%     X_s((i-2)*(450 + 1-256)+1 : (i-1)*(450 + 1-256)) = X(i,256:450);
%     Y_s((i-2)*(450 + 1-256)+1 : (i-1)*(450 + 1-256)) = Y(i,256:450);
%     Z_s((i-2)*(450 + 1-256)+1 : (i-1)*(450 + 1-256)) = Z(i,256:450);
%     % sample_mid((i-1)*(450 + 1-256)+1 : i*(450 + 1-256)) = sample_l(i,256:450);
% end
% sample_p=[];MID=[];
% for count_l = 1:898
%     if count_l >1
%         % sample_l11=sample_l1(m1_last(count_l-1) : m1_last(count_l));
%         % sample_l12=sample_l2(m2_last(count_l-1) : m2_last(count_l));
%         % sample_l13=sample_l3(m3_last(count_l-1) : m3_last(count_l));
%         % sample_l14=sample_l4(m4_last(count_l-1) : m4_last(count_l));
%         mid_ar = (682 * count_l  + 256) : (682 * count_l  + 450);
%         sample_p=[sample_p sample_l1(m1_last(count_l-1)+1 : m1_last(count_l)) sample_l2(m2_last(count_l-1)+1 : m2_last(count_l)) ...
%             mid_ar  sample_l3(m3_last(count_l-1)+1 : m3_last(count_l)) ...
%             sample_l4(m4_last(count_l-1)+1 : m4_last(count_l))];
%         pu=1;
%         point_num_in_frame(count_l) = numel( sample_l1(m1_last(count_l-1)+1 : m1_last(count_l))) + numel(sample_l2(m2_last(count_l-1)+1 : m2_last(count_l))) ...
%             + numel(mid_ar) + numel(sample_l3(m3_last(count_l-1)+1 : m3_last(count_l))) ...
%             + numel(sample_l4(m4_last(count_l-1)+1 : m4_last(count_l)));
%         %         MID=[MID; mid_ar];
%     elseif count_l ==1
%         mid_ar = (682 + 256) : (682 + 450);
%         sample_p = [sample_l1(1 : m1_last(count_l)) sample_l2(1: m2_last(count_l)) ...
%             mid_ar sample_l3(1 : m3_last(count_l)) ...
%             sample_l4(1 : m4_last(count_l))];
%         point_num_in_frame(1) = numel( sample_l1(1 : m1_last(1))) + numel(sample_l2(1 : m2_last(1))) ...
%             + numel(mid_ar) + numel(sample_l3(1 : m3_last(1))) + numel(sample_l4(1 : m4_last(1)));
%         pu2=1;
%         %         MID= mid_ar;
%     end
% end
% X1= [Xg1 Xg2 Xg3 Xg4 X_s];
% Y1= [Yg1 Yg2 Yg3 Yg4 Y_s];
% Z1= [Zg1 Zg2 Zg3 Zg4 Z_s];
% % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% frame_range = 654;
% % Completely without mirror case
% X1= X(1:frame_range,:);
% Y1= Y(1:frame_range,:);
% Z1= Z(1:frame_range,:);
% % % %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_mod = X1';
Y_mod = Y1';
Z_mod = Z1';
X_array = (X_mod(:))';
Y_array = (Y_mod(:))';
Z_array = (Z_mod(:))';


% for j=1: numel(X_array)
%     COrd(j,1) = X_array(j);
%     COrd(j,2) = Y_array(j);
%     COrd(j,3) = Z_array(j);
% end
% voxel coordinate database
Data_1 = BuildOctree(X_array',Y_array',Z_array',voxel_res);

voxel_res = 0.05; % 5cm side
xmax = max([X_array' Y_array' Z_array'], [], 1);
xmin = min([X_array' Y_array' Z_array'], [], 1);

% we need to set Xmin and Xmax based on the min and max range of the experimantal environment
% and based on how the obstacle or the robot moves corresponding positions shoulbe inducted in
% the voxel space.
xdel = (xmax-xmin)/2*.0005;
xmax = xmax+xdel;
xmin = xmin-xdel;
Lc = max(xmax-xmin);

XYZCenter  = (xmax+xmin)/2;
CubeCenter = xmin+Lc/2;
xmin       = xmin-(CubeCenter-XYZCenter);

Nlevel = round(log10(Lc/voxel_res)/log10(2));
twol  = 2^Nlevel;
two2l = twol^2;
dl    = Lc/twol;
% conversion to voxel coordinates
i      = floor((X_array'-xmin(1))/dl);
j      = floor((Y_array'-xmin(2))/dl);
k      = floor((Z_array'-xmin(3))/dl);

parent = two2l*k+twol*j+i;
groupcenter = [xmin(1)+(i+.5)*dl xmin(2)+(j+.5)*dl xmin(3)+(k+.5)*dl];

[iindex jindex kindex] = meshgrid(-1:1, -1:1, -1:1);
iindex = iindex(:);
jindex = jindex(:);
kindex = kindex(:);
% ... remove the self term, II = JJ = KK = 0.  The self term is not a neighbor of itself.
M = find(~(iindex == 0 & jindex == 0 & kindex == 0));
iindex = iindex(M);
jindex = jindex(M);
kindex = kindex(M);

[P11 P12 P13] = unique(i);
[P21 P22 P23] = unique(j);
[P31 P32 P33] = unique(k);

for count = 1:numel(i)
    summed(count) = i(count)+ j(count) + k(count);
end
summed_1 = summed;

c1 = 1;
i = single(i);
j = single(j);
k = single(k);

Big_mat = [i j k];
% unique combinations of i,j and k
unique_i = unique(i);
unique_j = unique(j);
unique_k = unique(k);

% from i,j,k to voxel ID computation
c2=1;c3 =1;
for count1 = min(unique_k): max(unique_k)
    for count2 = min(unique_j): max(unique_j)
        for count3 = min(unique_i): max(unique_i)
            ID(c2) = (count1-1) *(numel(unique_i)*numel(unique_j)) + (count2-1) * (numel(unique_i)) + count3;
            for count4 = 1: numel(i)
                if (count1 == k(count4)) && (count2 == j(count4)) && (count3 == i(count4))
                    ID_map(c2) = ID(c2);
                else ID_map(c2) = 0;
                end
            end
            c2 =c2+1;
        end
    end
end

% Obtain the repeated voxels accross all the frames
% voxel repeatation based on frames

%%%%%%%%%%%%%%%%%%%%%%%%% for mirrored config.%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% array_sp = []; c_ter=1;c1=1;
% for count5 = 1:numel(i)
%     %     L = find(summed == summed(count5));
%     %     [l1 l2] = find(unique_i == i(count5));
%     %     [l11 l12] = find(unique_j==j(count5));
%     %     [l21 l22] = find(unique_k==k(count5));
%     if summed(count5)~= 1000
%         mat_check = Big_mat -[i(count5) j(count5) k(count5)];
%         interim = mat_check';
%         mat_check_sum = abs(interim(1,:)) + abs(interim(2,:)) + (abs(interim(3,:)));
%         [l1 l2] = find(unique_i == i(count5));
%         [l11 l12] = find(unique_j == j(count5));
%         [l21 l22] = find(unique_k == k(count5));
%         [A1 B1] = find((mat_check_sum)==0);
%         mat_count{c1,1} = single(ceil(sample_p(B1)/682));
%         mat_count{c1,2} = i(count5);
%         mat_count{c1,3} = j(count5);
%         mat_count{c1,4} = k(count5);
%         mat_count{c1,5} = single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
%         summed(B1) = 1000;
%         c1 = c1+1;
%         clear mat_check
%     end
%     
%     if sum(point_num_in_frame(1:c_ter)) > count5
%         Id_cal =  single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
%         if Id_cal ==5234
%             array_sp = [array_sp 0];
%         else
%             array_sp = [array_sp Id_cal];
%         end
%         voxel_in_frame{c_ter,1} = c_ter;
%         voxel_in_frame{c_ter,2} = array_sp;
%         
%     elseif sum(point_num_in_frame(1:c_ter)) == count5
%         Id_cal =  single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
%         if Id_cal ==5234
%             array_sp = [array_sp 0];
%         else
%             array_sp = [array_sp Id_cal];
%         end
%         voxel_in_frame{c_ter,1} = c_ter;
%         voxel_in_frame{c_ter,2} = array_sp;
%         c_ter=c_ter+1;
%         clear array_sp;
%         array_sp = [];
%         %         X_min_center(ceil(count5/677)) = min(X_array(count5 - 677 +1 : count5));
%         %         Y_min_center(ceil(count5/677)) = min(Y_array(count5 - 677 +1 : count5));
%         %         Z_min_center(ceil(count5/677)) = min(Z_array(count5 - 677 +1 : count5));
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%% without mirror config.%%%%%%%%%%%%%%%%%%%%%%%%%
array_sp = [];
for count5 = 1:numel(i)
    L = find(summed == summed(count5));
        [l1 l2] = find(unique_i == i(count5));
        [l11 l12] = find(unique_j==j(count5));
        [l21 l22] = find(unique_k==k(count5));
            if summed(count5)~= 1000
                mat_check = Big_mat -[i(count5) j(count5) k(count5)];
                interim = mat_check';
                mat_check_sum = abs(interim(1,:)) + abs(interim(2,:)) + (abs(interim(3,:)));
                [l1 l2] = find(unique_i == i(count5));
                [l11 l12] = find(unique_j==j(count5));
                [l21 l22] = find(unique_k==k(count5));
                [A1 B1] = find((mat_check_sum)==0);
                mat_count{c1,1} = single(ceil(B1/677));
                mat_count{c1,2} = i(count5);
                mat_count{c1,3} = j(count5);
                mat_count{c1,4} = k(count5);
                mat_count{c1,5} = single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
                summed(B1) = 1000;
                c1 = c1+1;
                clear mat_check
            end
    if ceil((count5)/677) > floor((count5)/677)
        Id_cal =  single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
        if Id_cal ==5234
            array_sp = [array_sp 0];
        else
            array_sp = [array_sp Id_cal];
        end
        voxel_in_frame{ceil((count5)/677),1} = ceil((count5)/677);
        voxel_in_frame{ceil((count5)/677),2} = array_sp;
    else
        clear array_sp;
        array_sp = [];
%         X_min_center(ceil(count5/677)) = min(X_array(count5 - 677 +1 : count5));
%         Y_min_center(ceil(count5/677)) = min(Y_array(count5 - 677 +1 : count5));
%         Z_min_center(ceil(count5/677)) = min(Z_array(count5 - 677 +1 : count5));
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Segmentation of static voxels, thereby static region
% % static_tag = [];
% % for count= 1: size(voxel_in_frame,1) - 1
% %     intersection_a = intersect(voxel_in_frame{count,2}, voxel_in_frame{count+1, 2});
% %     for count2 = 1: numel(intersection_a)
% %         diff_checker = abs(intersection_a - intersection_a(count2));
% %         check_var1 = find( diff_checker ==1);
% %         check_var2 = find(diff_checker == numel(unique_i));
% %         check_var3 = find(diff_checker == numel(unique_i) * numel(unique_j);
% %
% %         static_voxel{c1} = [intersection_a(count2) intersection_a(check_var1) intersection_a(check_var2) intersection_a(check_var3)];
% %
% %     end
% %     static_tag = [static_tag intersection_a];
% % end



c3 =1;
voxel_database = [];
K_data = [mat_count{2:end,4}];
k_min = min(K_data);
k_filtered = find(K_data <=40);
voxel_ground_f=[];c5=1;
for count = 2: size(mat_count)
    voxel_database = [voxel_database mat_count{count,5}];
    % ground filtered voxel database
    voxel_ground_f = [voxel_ground_f mat_count{count,5}];
    if mat_count{count,4} <=51
        voxel_ground_f(c5) = [];
        c5 = c5-1;
    end
    c5 = c5+1;
end

for count6 = 1:numel(voxel_ground_f)
    diff_checker = abs(voxel_ground_f - voxel_ground_f(count6));
    check_var1 = find( diff_checker ==1);
    count_sp = 1 * numel(check_var1>=1);
    while count_sp <= numel(check_var1) && count_sp > 0
        addr = find(voxel_database==voxel_ground_f(check_var1(count_sp)));
        if abs(mat_count{addr+1,2} - mat_count{count6+1,2}) ~=1 && count6>1
            if mod(voxel_ground_f(count6), (numel(unique_i) * numel(unique_j))) ==0
                check_var1(count_sp) =[];
                % count_sp = count_sp - 1;
            elseif mod(voxel_ground_f(count6), numel(unique_i)) ==0
                check_var1(count_sp) =[];
                % count_sp = count_sp - 1;
            elseif mod(voxel_ground_f(count6), numel(unique_i)) ~=0
                check_var1(count_sp) =[];
                % count_sp = count_sp - 1;
            end
        else
            count_sp = count_sp + 1;
        end

    end

    check_var2 = find(diff_checker == numel(unique_i));
    check_var3 = find(diff_checker == numel(unique_i) * numel(unique_j));
    static_voxel{c3,1} = [voxel_ground_f(count6) voxel_ground_f(check_var1) voxel_ground_f(check_var2) voxel_ground_f(check_var3)];
    static_voxel{c3,2} = 0;
    c3 = c3 + 1;
end

c4 =  1; max_count = size(static_voxel,1);
inter2_arr=[]; static_voxel_1 = static_voxel;
for count7 = 1: max_count
    inter2_arr = static_voxel{count7};
    count8 =1;
    while count8 < max_count
        intersection_a = intersect(inter2_arr, static_voxel{count8});

        if intersection_a == 0
            intersection_a = [];
        end
        inter2_arr = union(inter2_arr, static_voxel{count8} * (numel(intersection_a)>0));
        if numel(intersection_a) ~= 0 && count8 ~=count7
            static_voxel(count8,:) = [];
            max_count = max_count -1;
            count8 = count8 - 1;
        end
        count8 = count8 +1;
    end
    static_voxel{count7,1} = inter2_arr;
    if count7== max_count
        break;
    end
    %     clear inter2_arr;
    % c4 = c4+1;
end

regenerate coordinate from ID
sub_s =97;
i_checker=[]; j_checker=[]; k_checker=[];
for count9 = 2 : numel(static_voxel{sub_s,1})
    i_checker(count9) = mat_count{find(voxel_database == static_voxel{sub_s,1}(count9)) + 1, 2};
    j_checker(count9) = mat_count{find(voxel_database == static_voxel{sub_s,1}(count9)) + 1, 3};
    k_checker(count9) = mat_count{find(voxel_database == static_voxel{sub_s,1}(count9)) + 1, 4};
end

figure
scatter3(i_checker, j_checker, k_checker,1, 'b','o');


for count_s = 1:numel(i)
    [l1 l2] = find(unique_i == i(count_s));
    [l11 l12] = find(unique_j == j(count_s));
    [l21 l22] = find(unique_k == k(count_s));
    ID_map(count_s) = single((l21 -1) * numel(unique_i) * numel(unique_j) + (l11-1) * (numel(unique_i)) + l1);
end
% motion estimation

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Dynamic voxels database WITHOUT MIRROR
D_obs = [];
% D_obs = [D_obs static_voxel{24:27,1}];
D_obs = [D_obs static_voxel{3,1}];
D_obs = [D_obs static_voxel{21,1}];
D_obs = [D_obs static_voxel{62,1}];
D_obs = [D_obs static_voxel{85,1}];
D_obs = [D_obs static_voxel{96,1}];
D_obs = [D_obs static_voxel{101,1}];
D_obs = [D_obs static_voxel{122,1}];
D_obs = [D_obs static_voxel{136,1}];
% D_obs = [D_obs static_voxel{139,1}];
% D_obs = [D_obs static_voxel{215,1}];
Dyn_obs = unique(D_obs);
Dyn_obs(1)=[];

% % static case
% D_obs = [];
% D_obs = [D_obs static_voxel{24:27,1}];
% D_obs = [D_obs static_voxel{177:188,1}];
% D_obs = [D_obs static_voxel{51,1}];
% D_obs = [D_obs static_voxel{97,1}];
% D_obs = [D_obs static_voxel{214,1}];
% Dyn_obs = unique(D_obs);
% Dyn_obs(1)=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Dynamic voxels database WITH MIRROR
% D_obs = [];
% % D_obs = [D_obs static_voxel{24:27,1}];
% % D_obs = [D_obs static_voxel{3,1}];
% % D_obs = [D_obs static_voxel{21,1}];
% D_obs = [D_obs static_voxel{51,1}];
% % D_obs = [D_obs static_voxel{85,1}];
% % D_obs = [D_obs static_voxel{96,1}];
% % D_obs = [D_obs static_voxel{101,1}];
% % D_obs = [D_obs static_voxel{122,1}];
% % D_obs = [D_obs static_voxel{136,1}];
% % D_obs = [D_obs static_voxel{139,1}];
% % D_obs = [D_obs static_voxel{215,1}];
% Dyn_obs = unique(D_obs);
% Dyn_obs(1)=[];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% D_obs = [];
% D_obs = [D_obs static_voxel{1:2,1}];
% D_obs = [D_obs static_voxel{25,1}];
% D_obs = [D_obs static_voxel{82,1}];
% D_obs = [D_obs static_voxel{85,1}];
% D_obs = [D_obs static_voxel{89,1}];
% D_obs = [D_obs static_voxel{98,1}];
% D_obs = [D_obs static_voxel{137,1}];
% Dyn_obs = unique(D_obs);
% Dyn_obs(1)=[];



% Dyn_base = [];
% for count11 = 1:numel(Dyn_obs)
%     Dyn_base = [Dyn_base find(voxel_database == Dyn_obs(count11))];
% end

% % % dynamic raw measurements in coordinates (m.)
% for count12 = 1:numel(D_obs)
%     Dyn_voxel_stamp{count12} =  find(Dyn_base(count12) == ID_map);  % voxel occurance in terms of time stamp
%     Dyn_voxel_frame{count12} = ceil(find(Dyn_base(count12) == ID_map)/677);
%     X_stamp{count12} = X_array(Dyn_voxel_stamp{count12});  % corresponding coordinates in the voxel
%     Y_stamp{count12} = Y_array(Dyn_voxel_stamp{count12});
%     Z_stamp{count12} = Z_array(Dyn_voxel_stamp{count12});
% end
count16 = 1; vox_x ={}; sample_period = 97656.2514552 * 10^ (-6); total_arr_x = []; total_arr_y = []; total_arr_z = [];
% % % for count13 = 1: numel(Dyn_obs)

for count13 = 1: size(voxel_in_frame,1)
    sample_time=[]; ar1 = []; ar2 =[]; ar3 =[];
    frame_s = size(voxel_in_frame{count13,2},2);
    for count14 = 1:frame_s
        temp_pos = find(voxel_in_frame{count13,2}(count14) == Dyn_obs);
        
        if numel(temp_pos) >0
            ar1 = [ar1 X_array(sum(point_num_in_frame(1:(count13-1))) + count14)];
            ar2 = [ar2 Y_array(sum(point_num_in_frame(1:(count13-1))) + count14)];
            ar3 = [ar3 Z_array(sum(point_num_in_frame(1:(count13-1))) + count14)];
            %             total_arr_x = [total_arr_x X_array((count13 - 1) *677 + temp_pos)];
            %             total_arr_y = [total_arr_y Y_array((count13 - 1) *677 + temp_pos)];
            %             total_arr_z = [total_arr_z Z_array((count13 - 1) *677 + temp_pos)];
            sample_time  = [sample_time  sample_period * sample_p(sum(point_num_in_frame(1:(count13-1))) + count14)];
        end
    end
    vox_x{count13} = ar1;
    vox_y{count13} = ar2;
    vox_z{count13} = ar3;
    sample_T{count13} = sample_time;
    
end

% for count13 = 1: size(voxel_in_frame,1)
%     sample_time=[]; ar1 = []; ar2 =[]; ar3 =[];
%     frame_s = size(voxel_in_frame{count13,2},2);
%     for count14 = 1:frame_s
%         temp_pos = find(voxel_in_frame{count13,2}(count14) == Dyn_obs);
%         
%         if numel(temp_pos) >0
%             ar1 = [ar1 X_array(677*(count13-1) + count14)];
%             ar2 = [ar2 Y_array(677*(count13-1) + count14)];
%             ar3 = [ar3 Z_array(677*(count13-1) + count14)];
%             %             total_arr_x = [total_arr_x X_array((count13 - 1) *677 + temp_pos)];
%             %             total_arr_y = [total_arr_y Y_array((count13 - 1) *677 + temp_pos)];
%             %             total_arr_z = [total_arr_z Z_array((count13 - 1) *677 + temp_pos)];
%             sample_time  = [sample_time  sample_period * 676*(count13-1) + count14];
%         end
%     end
%     vox_x{count13} = ar1;
%     vox_y{count13} = ar2;
%     vox_z{count13} = ar3;
%     sample_T{count13} = sample_time;
%     
% end


%
% end
% X_min_pos = find(min(total_arr_x) == total_arr_x);
% Y_min_pos = find(min(total_arr_y) == total_arr_y);
% X_max_pos = find(max(total_arr_x) == total_arr_x);
% Y_max_pos = find(max(total_arr_y) == total_arr_y);
% for count17 = 1:size(vox_x,2)
%     if numel(vox_x{count17}) > 0
%         total_arr_x = [total_arr_x vox_x{count17}];
%         total_arr_y = [total_arr_y vox_y{count17}];
%         total_arr_z = [total_arr_z vox_z{count17}];
%     end
% end

% figure
% for count17 = 1:size(vox_x,2)
%     if numel(vox_x{count17}) >0
%         scatter3(vox_x{count17}, vox_y{count17}, vox_z{count17} ,1, 'b','o');
%         hold on
%     end
% end

% data point interpolation for estimating transformation%%%%%%%%%%%%%%%%%%%
% for count19 = 1: size(vox_x,2)
%     cloud_s(count19) = numel(vox_x{count19});
% end
% max_size = max(cloud_s);
% for count19 = 1: size(vox_x,2)
%     if numel(vox_x{count19}) >0
%         interp_s=[vox_x{count19}(1:24); vox_y{count19}(1:24); vox_z{count19}(1:24)]';
%         CS = cat(1,0,cumsum(sqrt(sum(diff(interp_s,[],1).^2,2))));
%         dd = interp1(CS, interp_s, unique([CS(:)' linspace(0,CS(end),50)]),'cubic');
%         vox_x{count19} = dd(:,1:end);
%         vox_y{count19} = dd(:,2:end);
%         vox_z{count19} = dd(:,3:end);
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% intial velocity set for the motion estimation
v_init = 0.000104; beta_init = 91.1; % good intiial value
% v_init = 0.00013; beta_init = 91.1 - 0.25 ; % no mirror
% v_init = 0.001; beta_init = 91.1 - 0.15 ; % mirror case

% v_init = -0.000401; beta_init = 91.1 - 0.4; % good intiial value

frame_period = 100; c5 =1; pos_finder=[];
% figure

for count15 = 1:150
    x_accum= []; y_accum= []; z_accum = [];
    if count15<=150
        if numel(vox_x{count15})>0
            pos_finder = [pos_finder count15];
            for count16 = count15:-1:1
                if numel(vox_x{count16})>0
                    x_accum = [x_accum vox_x{count16} + v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{count16}(1))];
                    y_accum = [y_accum vox_y{count16} + v_init * sind(beta_init) * (sample_T{count15}(1) - sample_T{count16}(1))];
                    z_accum = [z_accum vox_z{count16}];
                end
            end
            % accumulated cloud in each frame
            vox_accum_X{c5} = x_accum;
            vox_accum_Y{c5} = y_accum;
            vox_accum_Z{c5} = z_accum;
            color_val = rand(1,3);
%             if count15==36
            figure
            plot3(vox_accum_X{c5}, vox_accum_Y{c5}, vox_accum_Z{c5}, 'MarkerSize', 2, ...
                'color', color_val,'Marker', 'o', 'LineStyle','none', 'MarkerFaceColor', color_val)
            title ('dynamic obstacle detection','FontWeight','bold','fontsize',12)
            xlabel ('x(m)','fontsize',12);
            ylabel ('y(m)','fontsize',12);
            zlabel ('z(m)','fontsize',12);
            %             axis([1.75 1.95 0.15 1.6 -0.1 0.25]);
            view(90,90)
            %             xticks(1.75:0.02:1.95);
            %             yticks(0.9:0.02:1.25);
            %             axis([1.75 2.05 0 2.5 0.1 0.25]);
%             hold on
            grid on
%                         pause(0.5)
%             end
            c5 = c5 + 1;
        end
    elseif count15>150 && count15 <=385
        
        if numel(vox_x{count15})>0
            pos_finder = [pos_finder count15];
            
            x_accum = [vox_x{count15} vox_accum_X{c5-1} - ...
                v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{pos_finder(c5 -1)}(1))];
            
            y_accum = [vox_y{count15} vox_accum_Y{c5-1} - ...
                v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{pos_finder(c5 -1)}(1))];
            
            z_accum = [vox_z{count15} vox_accum_Z{c5-1}];
            vox_accum_X{c5} = x_accum;
            vox_accum_Y{c5} = y_accum;
            vox_accum_Z{c5} = z_accum;
            color_val = rand(1,3);
            plot3(vox_accum_X{c5}, vox_accum_Y{c5}, vox_accum_Z{c5}, 'MarkerSize', 2, ...
                'color', color_val,'Marker', 'o', 'LineStyle','none', 'MarkerFaceColor', color_val)
            hold on
            grid on
            %                 axis([1.75 2 0 1.7 0.1 0.25]);
            c5 = c5 + 1;
            % accumulated cloud in each frame
        end
    end
end

% refined voxel representation of the inconsistent measurements
% voxel coordinate database

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
del_vel = 0, del_beta = 0; % optimal parameteres initialization
del_vel = -0.1:0.001:0.1;
del_beta = -0.3:0.01:0.3;
fine_res = 0.02; % 5cm side
p_occ = 0.9; p_emp = 1 - p_occ;

for count_3 = 1:150
    c5=1;
    if count_3<=150
        if numel(vox_x{count_3})>0
            pos_finder = [pos_finder count_3];
            for count_1 = 1: numel(del_vel)
                for count_2 = 1:numel(del_beta)
                    x_accum= []; y_accum= []; z_accum = [];
                    for count_4 = count_3:-1:1
                        if numel(vox_x{count_4})>0
                            x_accum = [x_accum vox_x{count_4} + (v_init + del_vel(count_1)) * cosd(beta_init + del_beta(count_2)) * (sample_T{count_3}(1) - sample_T{count_4}(1))];
                            y_accum = [y_accum vox_y{count_4} + (v_init + del_vel(count_1)) * sind(beta_init + del_beta(count_2)) * (sample_T{count_3}(1) - sample_T{count_4}(1))];
                            z_accum = [z_accum vox_z{count_4}];
                        end
                    end
                    % accumulated cloud in each frame
                    vox_accum_X{c5} = x_accum;
                    vox_accum_Y{c5} = y_accum;
                    vox_accum_Z{c5} = z_accum;
                    
                    
                    xmax = max([vox_accum_X{c5}' vox_accum_Y{c5}' vox_accum_Z{c5}'], [], 1);
                    xmin = min([vox_accum_X{c5}' vox_accum_Y{c5}' vox_accum_Z{c5}'], [], 1);
                    
                    % we need to set Xmin and Xmax based on the min and max range of the experimantal environment
                    % and based on how the obstacle or the robot moves corresponding positions shoulbe inducted in
                    % the voxel space.
                    xdel = (xmax-xmin)/2*.0005;
                    xmax = xmax+xdel;
                    xmin = xmin-xdel;
                    Lc = max(xmax-xmin);
                    
                    XYZCenter  = (xmax+xmin)/2;
                    CubeCenter = xmin+Lc/2;
                    xmin       = xmin-(CubeCenter-XYZCenter);
                    
                    Nlevel = round(log10(Lc/fine_res)/log10(2));
                    twol  = 2^Nlevel;
                    two2l = twol^2;
                    dl_1    = Lc/twol;
                    
                    i_prime = ceil((vox_accum_X{c5}' - xmin(1))/dl_1);
                    j_prime = ceil((vox_accum_Y{c5}' - xmin(2))/dl_1);
                    k_prime = ceil((vox_accum_Z{c5}' - xmin(3))/dl_1);
                    
                    i_prime_dup = i_prime; j_prime_dup = j_prime;
                    
                    unique_i_p = unique(i_prime);
                    unique_j_p = unique(j_prime);
                    unique_k_p = unique(k_prime);
                    
                    ij_mat = [i_prime j_prime k_prime];
                    ij_mat_dup = ij_mat;
                    match_finder=[]; vec = [0 0];
                    for count18 = 1: numel(i_prime)
                        ij_mat_temp =[];
                        ij_mat_temp = ij_mat - ij_mat(count18,:);
                        
                        temp_1 = find(sum(abs(ij_mat_temp')) == 0);
                        temp_1(temp_1== count18)=[];
                        match_finder = [match_finder temp_1];
                    end
                    i_prime(match_finder) = [];
                    % i_p_finder = []; j_p_finder =[];
                    % for count18 = 1: numel(unique_i_p)
                    %     % if (i_prime(count18) + (j_prime - 1) * numel(unique(i_prime))) ==
                    %     i_p_finder = [i_p_finder; (find(i_prime == unique_i_p(count18)))];
                    % end
                    
                    % for count19 = 1: numel(unique_j_p)
                    %     j_p_finder = [j_p_finder; find(j_prime == unique_j_p(count19))];
                    % end
                    % i_j_match = intersect(i_p_finder, j_p_finder);
                    % unique i_prime and j_prime combinations
                    % i_prime(i_j_match) = [];
                    
                    % Entropy computation
                    Entropy_dyn(count_3, c5) = - numel(i_prime) * (p_occ * log2(p_occ) + p_emp * log2(p_emp));
                    delv(c5) =  del_vel(count_1);
                    delphi(c5) = del_beta(count_2);
                    % color_val = rand(1,3);
                    % plot3(vox_accum_X{c5}, vox_accum_Y{c5}, vox_accum_Z{c5}, 'MarkerSize', 2, ...
                    %     'color', color_val,'Marker', 'o', 'LineStyle','none', 'MarkerFaceColor', color_val)
                    % title ('dynamic obstacle detection','FontWeight','bold','fontsize',12)
                    % xlabel ('x (m)','fontsize',12);
                    % ylabel ('y(m)','fontsize',12);
                    % zlabel ('z(m)','fontsize',12);
                    % axis([1.75 1.95 0.15 1.6 -0.1 0.25]);
                    % view(90,90)
                    % %             xticks(1.75:0.02:1.95);
                    % %             yticks(0.9:0.02:1.25);
                    % %             axis([1.75 2.05 0 2.5 0.1 0.25]);
                    % hold on
                    % grid on
                    % pause(1)
                    c5 = c5 + 1;
                end
            end
        end
    end
end

% for count19 = 1:numel(del_vel) * numel(del_beta)
%     min_entropy(count19) = sum(Entropy_dyn(:,count19));
% end


% for count15 = 1:120
%     x_accum= []; y_accum= []; z_accum = [];
%     if count15<=150
%         if numel(vox_x{count15})>0
%             pos_finder = [pos_finder count15];
%             for count16 = count15:-1:1
%                 if numel(vox_x{count16})>0
%                     x_accum = [x_accum vox_x{count16} + v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{count16}(1))];
%                     y_accum = [y_accum vox_y{count16} + v_init * sind(beta_init) * (sample_T{count15}(1) - sample_T{count16}(1))];
%                     z_accum = [z_accum vox_z{count16}];
%                 end
%             end
%             % accumulated cloud in each frame
%             vox_accum_X{c5} = x_accum;
%             vox_accum_Y{c5} = y_accum;
%             vox_accum_Z{c5} = z_accum;
%             color_val = rand(1,3);

%             % plot3(vox_accum_X{c5}, vox_accum_Y{c5}, vox_accum_Z{c5}, 'MarkerSize', 2, ...
%             %     'color', color_val,'Marker', 'o', 'LineStyle','none', 'MarkerFaceColor', color_val)
%             % title ('dynamic obstacle detection','FontWeight','bold','fontsize',12)
%             % xlabel ('x (m)','fontsize',12);
%             % ylabel ('y(m)','fontsize',12);
%             % zlabel ('z(m)','fontsize',12);
%             % axis([1.75 1.95 0.15 1.6 -0.1 0.25]);
%             % view(90,90)
%             % %             xticks(1.75:0.02:1.95);
%             % %             yticks(0.9:0.02:1.25);
%             % %             axis([1.75 2.05 0 2.5 0.1 0.25]);
%             % hold on
%             % grid on
%             % pause(1)
%             c5 = c5 + 1;
%         end


%         fine_res = 0.02; % 5cm side
%         % Data_1 = BuildOctree(X_array',Y_array',Z_array',voxel_res);

%         xmax = max([vox_accum_X{26}' vox_accum_Y{26}' vox_accum_Z{26}'], [], 1);
%         xmin = min([vox_accum_X{26}' vox_accum_Y{26}' vox_accum_Z{26}'], [], 1);

%         % we need to set Xmin and Xmax based on the min and max range of the experimantal environment
%         % and based on how the obstacle or the robot moves corresponding positions shoulbe inducted in
%         % the voxel space.
%         xdel = (xmax-xmin)/2*.0005;
%         xmax = xmax+xdel;
%         xmin = xmin-xdel;
%         Lc = max(xmax-xmin);

%         XYZCenter  = (xmax+xmin)/2;
%         CubeCenter = xmin+Lc/2;
%         xmin       = xmin-(CubeCenter-XYZCenter);

%         Nlevel = round(log10(Lc/fine_res)/log10(2));
%         twol  = 2^Nlevel;
%         two2l = twol^2;
%         dl_1    = Lc/twol;

%         i_prime = floor((vox_accum_X{26}' - xmin(1))/dl_1);
%         j_prime = floor((vox_accum_Y{26}' - xmin(2))/dl_1);
%         k_prime = floor((vox_accum_Z{26}' - xmin(3))/dl_1);

%         parent = two2l*k+twol*j+i;
%         groupcenter = [xmin(1)+(i+.5)*dl_1 xmin(2)+(j+.5)*dl_1 xmin(3)+(k+.5)*dl_1];

%         unique_i_p = unique(i_prime);
%         unique_j_p = unique(j_prime);

%         j_p_finder = []; j_p_finder =[];
%         for count18 = 1: numel(unique_i_p)
%             % if (i_prime(count18) + (j_prime - 1) * numel(unique(i_prime))) ==
%             i_p_finder = [i_p_finder find(i_prime == unique_i_p(count18))];
%         end

%         for count19 = 1: numel(unique_j_p)
%             j_p_finder = [j_p_finder find(j_prime == unique_j_p(count19))];
%         end
%         i_j_match = intersect(i_p_finder, j_p_finder);
%         % unique i_prime and j_prime combinations
%         i_prime(i_j_match) = [];
%         % elseif count15>150 && count15 <=385

%         %     if numel(vox_x{count15})>0
%         %         pos_finder = [pos_finder count15];

%         %         x_accum = [vox_x{count15} vox_accum_X{c5-1} - ...
%         %             v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{pos_finder(c5 -1)}(1))];

%         %         y_accum = [vox_y{count15} vox_accum_Y{c5-1} - ...
%         %             v_init * cosd(beta_init) * (sample_T{count15}(1) - sample_T{pos_finder(c5 -1)}(1))];

%         %         z_accum = [vox_z{count15} vox_accum_Z{c5-1}];
%         %         vox_accum_X{c5} = x_accum;
%         %         vox_accum_Y{c5} = y_accum;
%         %         vox_accum_Z{c5} = z_accum;
%         %         color_val = rand(1,3);
%         %         plot3(vox_accum_X{c5}, vox_accum_Y{c5}, vox_accum_Z{c5}, 'MarkerSize', 2, ...
%         %             'color', color_val,'Marker', 'o', 'LineStyle','none', 'MarkerFaceColor', color_val)
%         %         hold on
%         %         grid on
%         %         %                 axis([1.75 2 0 1.7 0.1 0.25]);
%         %         c5 = c5 + 1;
%         %         % accumulated cloud in each frame
%         %     end
%     end
% end




% for count4 = 1: size(mat_count,1)
%     count6=1;
%     voxel_neighbour_i(count4,1) = mat_count{count4,2};
%     voxel_neighbour_j(count4,1) = mat_count{count4,3};
%     voxel_neighbour_k(count4,1) = mat_count{count4,4};

%     for count5 = 1:size(mat_count,1)

%     if abs(mat_count{count4,2} - mat_count{count5,2}) <=1 && abs(mat_count{count4,3} -mat_count{count5,3}) <=1 && abs(mat_count{count4,4} -mat_count{count5,4}) <=1
%         count6 = count6 + 1;
%         voxel_neighbour_i(count4,count6) = mat_count{count5,2};
%         voxel_neighbour_j(count4,count6) = mat_count{count5,3};
%         voxel_neighbour_k(count4,count6) = mat_count{count5,4};
%     end
%     end
% end
% region_grown_i= zeros(27,1);region_grown_j= zeros(27,1);region_grown_k= zeros(27,1);

% for count7 = 1: size(voxel_neighbour_i)
%     region_grown_i(1:size(intersect(voxel_neighbour_i(count7,:), voxel_neighbour_i(1:end,:))),count7) = intersect(voxel_neighbour_i(count7,:), voxel_neighbour_i(1:end,:));
%     region_grown_j(1:size(intersect(voxel_neighbour_j(count7,:), voxel_neighbour_j(1:end,:))),count7) = intersect(voxel_neighbour_j(count7,:), voxel_neighbour_j(1:end,:));
%     region_grown_k(1:size(intersect(voxel_neighbour_k(count7,:), voxel_neighbour_k(1:end,:))),count7) = intersect(voxel_neighbour_k(count7,:), voxel_neighbour_k(1:end,:));
%     global_neighbour_i()
% end


c2 =1; int_array=[]; int_array=[]; int_array=[];
for count9 = 2: size(mat_count,1)
    % unique frames wheere the voxel are hit
    for count10 = 1: numel(unique_frames{count9-1})
        for count11 = 2:size(mat_count,1)
            [l1 l2 l3] = find(unique_frames{count11-1}, unique_frames{count9-1}(count10));
            if l3==1
                int_array_i = [int_array_i mat_count{count9,2}];
                % voxels hit in a frame accross all the frames
                voxel_frame(c2,1) = unique_frames{count9-1}(count10);
                voxel_frame_ID(c2,2) = [voxel_frame_ID int_array_i];
            end
        end
        c2 = c2+1;
    end
end

% for count7 = 1: size(voxel_neighbour_i)
%     for count8 = 1: nnz(region_grown_i(:, count7))
%         if region_grown_i(count8 + 1, count7) == voxel_neighbour_i(count7,:) &&

%     end
%     if voxel_neighbour_i()
% end

% % K-nearest neighbor check based on euclidian distance

% for count8 =1: size(mat_count,1)
%     if distance_eu(mat_count{count8,3}, mat_count{count8,4}, mat_count{:,3}, mat_count{:,4}
% end






% for count2 = 1: c1
%     for count3 = 2:numel(mat_count{c1,1})
%         if mat_count{c1, count3} -mat_count{c1,1 > 677}

%     end
% end

%               pts = rand(200,3);
%        OT = OcTree(COrd,'binCapacity',10,'maxDepth',10,'style','weighted');
%        OT.shrink
%        figure
%        boxH = OT.plot;
%        cols = lines(OT.BinCount);
%        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
%        for i = 1:OT.BinCount
%            set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i))
%            doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:))
%        end
%        axis image, view(3)
