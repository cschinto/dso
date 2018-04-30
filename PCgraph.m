clear all
close all

%% Key Frames Coordinates
KFcoord = load('ressources/result.txt') ;

QuatKF = [];
CoordKF = [];
for i=1:length(KFcoord)
    % Quaternion : w x y z
    QuatKF = [QuatKF; KFcoord(i,9) KFcoord(i,6) KFcoord(i,7) KFcoord(i,8)];
    
    % Transpose: x y z
    CoordKF = [CoordKF; KFcoord(i,3) KFcoord(i,4) KFcoord(i,5) ];
    
end

%% KF orientation
% To find the correct orientation of the key frames

KFrot = [];
X = [];
Y = [] ;
Z = [];
for i=1:length(QuatKF)
    Rotmat{i} = [quat2rotm(QuatKF(i, :))];  % Quaternion to rotation matrix
    KFrot = [ KFrot Rotmat{1,i}(:,:)];
    %x = [Rotmat{1,i}(:,1)]
    X = [X Rotmat{1,i}(:,1)];
    Y = [Y Rotmat{1,i}(:,2)];
    Z = [Z Rotmat{1,i}(:,3)];
end

% figure(3)
% quiver3(CoordKF(:,1), CoordKF(:,2), CoordKF(:,3), X(1,:)', X(2,:)', X(3,:)','b')
% hold on 
% plot3(CoordKF(:,1), CoordKF(:,2), CoordKF(:,3), 'r')
% title('Orientation: X')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

% figure(4)
% quiver3(CoordKF(:,1), CoordKF(:,2), CoordKF(:,3), Y(1,:)', Y(2,:)', Y(3,:)', 'b')
% hold on 
% plot3(CoordKF(:,1), CoordKF(:,2), CoordKF(:,3), 'r')
% title('Orientation: Y')
% xlabel('X')
% ylabel('Y')
% zlabel('Z')

figure(5)
for i =1:100:length(CoordKF)
    quiver3(CoordKF(i,1), CoordKF(i,2), CoordKF(i,3), Z(1,i)', Z(2,i)', Z(3,i)','b')
    hold on 
    plot3(CoordKF(i,1), CoordKF(i,2), CoordKF(i,3), '-r')
end
title('Orientation: Z')
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Point Cloud coordinates

% Retrieve the ID of the KF from the PC filename
d = dir('ressources/PC/*.txt'); 
numb = length(d);
PCnames = ' ';
PCKFiddes = [];
for i=1:numb
    filename = d(i).name;
    [~,name] = fileparts(filename);
    %PCnames = [PCnames,' ',name];
    part = '\_';
    PCsplit = regexp(name, part, 'split');
    PCKFiddes = [PCKFiddes str2num(PCsplit{2})];
end
PCKFid = sort(PCKFiddes);

% Only load the PC matching the KF id
for i=1:numb
   for j=1:length(KFcoord)
        if (KFcoord(j, 1) == PCKFid(i))
            PCcoord{j} = [load(['ressources/PC/PointCloudKF_' num2str(PCKFid(i)) '.txt'])];
        end
   end
end


%% Coordinates in "world" (Rotation + Translation)

for i=1:length(QuatKF)
    q = QuatKF(i, :);
    
    vect3 = [];
    FinalPCKF = [];
    for j=1:length(PCcoord{1,i})
        % Rotation
        vect3 = [vect3; quatrotate(q, PCcoord{1,i}(j,:))] ;
        % Transpose
        FinalPCKF = [FinalPCKF ; vect3(j,:)+CoordKF(i, :)];
    end
    FinalPC{i} = FinalPCKF ;
end

%% Figure

% every 100 KF plotted
figure
for i=1:100:length(FinalPC)
    scatter3(FinalPC{1,i}(:,1),FinalPC{1,i}(:,2),FinalPC{1,i}(:,3), 1,FinalPC{1,i}(:,3))
    hold on 
end
plot3(CoordKF(:,1), CoordKF(:,2), CoordKF(:,3), 'r')
xlabel('x')
ylabel('y')
zlabel('z')
title('PC and KF (robot path)')
