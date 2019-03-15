clear all; close all; clc
dir = '../data';
oxts = loadOxtsliteData(dir);
pose = convertOxtsToPose(oxts);

tracklets = readTracklets([dir '/tracklet_labels.xml']);

map = robotics.OccupancyMap3D(1.5);

% global l b h;
l=3;b=2; h = 1.2;%size of car

for n = 1:length(pose)
    sensorPose = getSP(n, pose); % car pose
    pointCloud = getPC(n, dir); % point cloud
    
    insertPointCloud(map, sensorPose, pointCloud,  150); % Update Occupancy Grid
    
    
    % load and display scene image
    subplot(2,2,1:2) 
    cam = 2;
    img = imread(sprintf('%s/image_%02d/data/%010d.png',dir,cam,n-1));
    imshow(img);
    title('Scene Image')
    
    % display real time point cloud
    subplot(2,2,3) 
    pcshow(pointCloud)
    hold on
    set(gca, 'View', [0, 90])
% % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     
    xlim([-60,60])
    ylim([-60,60])
    % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % % %     

    title('Instantaneous Point Cloud')
    
    
    plot3([-l/2,l/2,l/2,-l/2,-l/2],[-b/2,-b/2,b/2,b/2,-b/2],[0,0,0,0,0],'b','LineWidth',1);
    xlabel('X[meters]')
    ylabel('Y[meters]')
    hold off
%     pbaspect([1,1,1])
    
    
%     Display current occupancy grid map
    subplot(2,2,4)
    show(map); % Display occupancy grid
    hold on;
%     drawTrackletBoxes(i, tracklets, pose); % Display tracklets
    traj(n, pose); % Display path and car
    set(gca, 'View', [0, 90])
    hold off;
%     pbaspect([1,1,1])
    pause(0.1);
    F(n) = getframe(gcf);
end


writerObj = VideoWriter('occupancy_grid_mapping.avi');
writerObj.FrameRate = 10;
% open the video writer
open(writerObj);
% write the frames to the video
for n=1:length(F)
    writeVideo(writerObj, F(n));
end
% close the writer object
close(writerObj);