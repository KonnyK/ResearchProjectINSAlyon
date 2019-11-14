world = zeros(20,20,20); % 0=libre, 1=occupée -1=inconnu

cube1 = cube([5,5,1], [15,15,11]);
cube2 = cube([3,3,1], [5,5,12]);
cube3 = cube([7,7,11], [12,12,16]);
cube4 = cube([20,20,20], [18,18,18]);
world = insert(world, [cube1;cube2;cube3;cube4], 1);

drone_pos = [5,5,20];
drone_maxDist = 10;
siz = size(world);
seenWorld = -ones(siz);

X = drone_pos(1);
Y = drone_pos(2);
Z = drone_pos(3);
for x = max(drone_pos(1)-drone_maxDist, 1):min(drone_pos(1)+drone_maxDist, siz(1))
    for y = max(drone_pos(2)-drone_maxDist, 1):min(drone_pos(2)+drone_maxDist, siz(2))
        for z = max(drone_pos(3)-drone_maxDist, 1):drone_pos(3)
            if sqrt((x-drone_pos(1))^2 + (y-drone_pos(2))^2 + (z-drone_pos(3))^2) <= drone_maxDist
                seenWorld(x,y,z) = world(x,y,z); 
                if seenWorld(x,y,z) == 1
                    X = [X;x];
                    Y = [Y;y];
                    Z = [Z;z];
                end
            end
        end
    end
end
scale = 500 * ones(length(X),1);
scale(1) = 100;
scatter3(X,Y,Z,scale,'filled','s');
axis([1,siz(1),1,siz(2),1,siz(3)]);


function c = cube(P1, P2)
    points = zeros(0,3);
    for x = P1(1,1):P2(1,1)
        for y = P1(1,2):P2(1,2)
            for z = P1(1,3):P2(1,3)
                points = [points ; [x,y,z]];
            end    
        end
    end
    c = points;
end

function ins = insert(map, pointcloud, value)
    for i = 1:length(pointcloud)
        map(pointcloud(i, 1), pointcloud(i, 2), pointcloud(i, 3)) = value;
    end
    ins = map;
end