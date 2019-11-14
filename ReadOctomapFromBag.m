bag = rosbag('C:/Users/konra/Desktop/Dokumente/Studium/MatLab/Research Project INSA Lyon/rtabmap.bag');
bagInfo = rosbag('info','C:/Users/konra/Desktop/Dokumente/Studium/MatLab/Research Project INSA Lyon/rtabmap.bag');
rosbag info 'C:/Users/konra/Desktop/Dokumente/Studium/MatLab/Research Project INSA Lyon/rtabmap.bag'
bSel1 = select(bag,'Topic','rtabmap/octomap_occupied_space');
Point_msgs = readMessages(bSel1,'DataFormat','BagSelection');
bSel2 = select(bag,'Topic','rtabmap/localization_pose');
Pose_msgs = readMessages(bSel2,'DataFormat','BagSelection');

map = occupancyMap3D;

j = 1;
stoplength = length(Point_msgs);
for i = 1:stoplength
    pointInfo = Point_msgs{i};
    poseInfo = Pose_msgs{j+1};
    if pointInfo.Header.Stamp.Nsec + pointInfo.Header.Stamp.Sec > poseInfo.Header.Stamp.Nsec + poseInfo.Header.Stamp.Sec
        j = j+1;
    end
    pos = Pose_msgs{j}.Pose.Pose.Position;
    ori = Pose_msgs{j}.Pose.Pose.Orientation;
    currentpose = [pos.X, pos.Y, pos.Z, ori.W, ori.X, ori.Y, ori.Z];
    insertPointCloud(map, currentpose, pointInfo.readXYZ, 100);
    %show(map);
    %pause(0.2);
%    map{i} = readOccupancyMap3D(msg);
end    
figure;
show(map);