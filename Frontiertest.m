world = zeros(200,200,200); % 0=libre, 1=occupée -1=inconnu

cube1 = cube([50,50,10], [150,150,110]);
cube2 = cube([30,30,10], [50,50,120]);
cube3 = cube([70,70,110], [120,120,160]);
cube4 = cube([200,200,200], [180,180,180]);
world = insert(world, [cube1;cube2;cube3;cube4], 1);

drone_pos = [70,50,180];
drone_maxDist = 500;
siz = size(world);
seenWorld = -ones(siz);

X1 = drone_pos(1);
Y1 = drone_pos(2);
Z1 = drone_pos(3);
X2 = zeros(0,1);
Y2 = zeros(0,1);
Z2 = zeros(0,1);

for x = max(drone_pos(1)-drone_maxDist, 1):min(drone_pos(1)+drone_maxDist, siz(1))
    for y = max(drone_pos(2)-drone_maxDist, 1):min(drone_pos(2)+drone_maxDist, siz(2))
        for z = max(drone_pos(3)-drone_maxDist, 1):drone_pos(3)
            rc = raycast(world, [x,y,z], drone_pos, 3);
            seeable = (rc(1) == drone_pos(1) && rc(2) == drone_pos(2) && rc(3) == drone_pos(3));
            if distance([x,y,z], drone_pos) <= drone_maxDist && seeable
                seenWorld(x,y,z) = world(x,y,z); 
                if seenWorld(x,y,z) == 1
                    X1 = [X1;x];
                    Y1 = [Y1;y];
                    Z1 = [Z1;z];
                end
            end
            if (world(x,y,z) == 1)
                X2 = [X2;x];
                Y2 = [Y2;y];
                Z2 = [Z2;z];
            end
        end
    end
end
figure;
scale = 20 * ones(length(X1),1);
scale(1) = 100;
scatter3(X1,Y1,Z1,scale,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);

figure;
scale = 20 * ones(length(X2),1);
scatter3(X2,Y2,Z2,scale,'filled','o');
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

function ray = raycast(map, From, To, depth)
    if (depth <= 0)
        ray = From;
        return;
    end
    Dir = (To-From)/max(abs(To-From));
    if (min(From == To) == 1)
        ray = From;
        return;
    end
    Next = [From(1)+Dir(1), From(2)+Dir(2), From(3)+Dir(3)];
    while not(min(Next == To) == 1 || distance(Next, From) >= distance(From, To) || map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        Next = [Next(1)+Dir(1), Next(2)+Dir(2), Next(3)+Dir(3)];
    end
    if (min(Next == To) == 1 || distance(Next, From) >= distance(From, To))
        ray = To;
    elseif (map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray = raycast(map, Next, To, depth-1);
    end
end

function dist = distance(P1, P2)
    dist = sqrt((P1(1)-P2(1))^2 + (P1(2)-P2(2))^2 + (P1(3)-P2(3))^2);
end
