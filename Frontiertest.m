world = zeros(200,200,200); % 0=libre, 1=occupée -1=inconnu

%cube1 = cube([50,50,10], [150,150,110]);
%cube2 = cube([30,30,10], [50,50,120]);
%cube3 = cube([70,70,110], [120,120,160]);
%cube4 = cube([200,200,200], [180,180,1]);
world = insert(world, [cube1;cube2;cube3;cube4], 1);

drone_pos = [60,40,170];
drone_maxDist = 500;
siz = size(world);
seenWorld = -ones(siz);

X1 = zeros(length(FOV)+1,1);
Y1 = zeros(length(FOV)+1,1);
Z1 = zeros(length(FOV)+1,1);
X1(1) = drone_pos(1);
Y1(1) = drone_pos(2);
Z1(1) = drone_pos(3);
X2 = zeros(siz(1)*siz(2)*siz(3),1);
Y2 = zeros(siz(1)*siz(2)*siz(3),1);
Z2 = zeros(siz(1)*siz(2)*siz(3),1);

FOV = demisphere(50, drone_pos);
i=2;
for index = 1:length(FOV)
    rc = raycast(world, drone_pos, [FOV(index,1),FOV(index,2),FOV(index,3)],drone_maxDist);
    if not(min(rc == drone_pos) == 1)
        seenWorld(rc(1), rc(2), rc(3)) = 1;
        i = i+1;
        X1(i) = rc(1);
        Y1(i) = rc(2);
        Z1(i) = rc(3);
    end
    free = (rc(1) == drone_pos(1) && rc(2) == drone_pos(2) && rc(3) == drone_pos(3));
end

j=1;
for x = 1:siz(1)
    for y = 1:siz(2)
        for z = 1:siz(3)
            if (world(x,y,z) == 1)
                X2(j) = x;
                Y2(j) = y;
                Z2(j) = z;
            end
            j=j+1;
        end
    end
end
X1((i+1):length(X1)) = [];
Y1((i+1):length(Y1)) = [];
Z1((i+1):length(Z1)) = [];
X2((j+1):length(X2)) = [];
Y2((j+1):length(Y2)) = [];
Z2((j+1):length(Z2)) = [];
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
    points = zeros(abs(P1(1,1)-P2(1,1))*abs(P1(1,2)-P2(1,2))*abs(P1(1,3)-P2(1,3)),3);
    j = 1;
    for x = min(P1(1,1),P2(1,1)):max(P1(1,1),P2(1,1))
        for y = min(P1(1,2),P2(1,2)):max(P1(1,2),P2(1,2))
            for z = min(P1(1,3),P2(1,3)):max(P1(1,3),P2(1,3))
                points(j,1) = x;
                points(j,2) = y;
                points(j,3) = z;
                j = j+1;
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

function ray = raycast(map, From, Towards, maxDist)
    Dir = (Towards-From)/max(abs(Towards-From));
    if (min(From == Towards) == 1)
        ray = From;
        return;
    end
    Next = [From(1)+Dir(1), From(2)+Dir(2), From(3)+Dir(3)];
    while not(distance(Next, From) >= maxDist || map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        Next = Next + Dir;
        limit = size(map);
        if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
            ray = From;
            return;
        end
    end
    if distance(Next, From) >= maxDist
        ray = From;
    elseif (map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray = round(Next);
    end
end

function dist = distance(P1, P2)
    dist = sqrt((P1(1)-P2(1))^2 + (P1(2)-P2(2))^2 + (P1(3)-P2(3))^2);
end

function demi = demisphere(radius, pos)
    demi = zeros(0,3);
    for x = pos(1)-radius:pos(1)+radius
        for y = pos(2)-radius:pos(2)+radius
            for z = pos(3)-radius:pos(3)
                if round(distance([x,y,z], pos)) == radius
                    demi = [demi;[x,y,z]];
                end
            end
        end
    end
    %figure;
    %scatter3(demi(:,1), demi(:,2), demi(:,3));
end
