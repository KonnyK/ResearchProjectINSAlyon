world = zeros(20,20,20); % 0=libre, 1=occupée -1=inconnu

cube1 = cube([1,1,1], [5,5,5]);
world = insert(world, cube1, 1);

drone_pos = [3,3,10];
drone_maxDist = 10;
siz = size(world);
seenWorld = -ones(siz);

for x = max(drone_pos(1)-drone_maxDist, 1):min(drone_pos(1)+drone_maxDist, siz(1))
    for y = max(drone_pos(2)-drone_maxDist, 1):min(drone_pos(2)+drone_maxDist, siz(2))
        for z = 1:20
            if sqrt((x-drone_pos(1))^2 + (y-drone_pos(2))^2 + (z-drone_pos(3))^2) <= drone_maxDist
                seenWorld(x,y,z) = world(x,y,z); 
                if seenWorld(x,y,z) == 1
                    plot3(x,y,z);
                end
            end
        end
    end
end


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