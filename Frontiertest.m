
world = zeros(200,200,200); % 0=libre, 1=occupée -1=inconnu

cube1 = cube([50,50,80], [150,150,110]);
cube2 = cube([30,30,10], [50,50,120]);
cube3 = cube([70,70,110], [120,120,160]);
cube4 = cube([200,200,200], [180,180,1]);
world = insert(world, [cube1;cube2;cube3;cube4], 1);

drone_pos = [60,40,170];
drone_maxDist = 500;
siz = size(world);
seenWorld = -ones(siz);
FOV = demisphere(150, drone_pos);

seen = zeros(length(FOV)+1,3);
seen(1,:) = drone_pos;
unseen = zeros(siz(1)*siz(2)*siz(3),3);
front = zeros(round((2/3)*pi*drone_maxDist^3),3);
free = zeros(round((2/3)*pi*drone_maxDist^3),3);


for index = 1:length(FOV)
    rc = raycast(world, drone_pos, [FOV(index,1),FOV(index,2),FOV(index,3)],drone_maxDist);
    for r = 1:length(rc)
        seenWorld(rc(r,1), rc(r,2), rc(r,3)) = world(rc(r,1), rc(r,2), rc(r,3));
    end
end

fo=1;
fe=1;
s=2;
u=1;
for x = 1:siz(1)
    for y = 1:siz(2)
        for z = 1:siz(3)
            if isFrontier(seenWorld, [x,y,z])
                front(fo,:) = [x,y,z];
                fo = fo+1;
            end
            if seenWorld(x,y,z) == 1
                seen(s,:) = [x,y,z];
                s = s+1;
            elseif world(x,y,z) == 1
                unseen(u,:) = [x,y,z];
                u = u+1;
            elseif seenWorld(x,y,z) == 0
                free(fe,:) = [x,y,z];
                fe = fe+1;
            end
        end
    end
end

seen(s:length(seen),:) = [];
unseen(u:length(unseen),:) = [];
front(fo:length(front),:) = [];
free(fe:length(free),:) = [];

figure;
draw = [seen;unseen];
Length = length(draw);
scale = 5 * ones(Length,1);
scale(1) = 100;
scale(length(seen)+1:Length) = 10;
color = 0.5 * ones(Length, 3);
color(1,:) = [1,0,0];
color(2:length(seen),:) = repmat([0,0,1],length(seen)-1,1);
scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);

figure;
draw = [seen;front];
Length = length(draw);
scale = 5 * ones(Length,1);
scale(1) = 100;
color = 0.5 * ones(Length, 3);
color(1,:) = [1,0,0];
color(2:length(seen),:) = repmat([0,0,1],length(seen)-1,1);
color(length(seen)+1:Length,:) = repmat([1,0.5,0],length(color)-length(seen),1);
scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);

function points = cube(P1, P2)
    points = zeros(abs(P1(1,1)-P2(1,1))*abs(P1(1,2)-P2(1,2))*abs(P1(1,3)-P2(1,3)),3);
    j = 1;
    for x = min(P1(1,1),P2(1,1)):max(P1(1,1),P2(1,1))
        for y = min(P1(1,2),P2(1,2)):max(P1(1,2),P2(1,2))
            for z = min(P1(1,3),P2(1,3)):max(P1(1,3),P2(1,3))
                points(j,:) = [x,y,z];
                j = j+1;
            end    
        end
    end
    points(j:length(points),:) = [];
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
    ray = zeros(1.5*maxDist, 3);
    i = 1;
    Next = [From(1)+Dir(1), From(2)+Dir(2), From(3)+Dir(3)];
    while not(distance(Next, From) >= maxDist || map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
        i = i+1;
        Next = Next + Dir;
        limit = size(map);
        if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
            ray(i:length(ray),:) = [];
            return;
        end
    end
    if distance(Next, From) >= maxDist
        ray(i,:) = From;
    elseif (map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
    end
    ray(i+1:length(ray),:) = [];
end

function dist = distance(P1, P2)
    dist = sqrt((P1(1)-P2(1))^2 + (P1(2)-P2(2))^2 + (P1(3)-P2(3))^2);
end

function demi = demisphere(radius, pos)
    demi = zeros(round(4*pi*radius^2),3);
    i = 1;
    for x = pos(1)-radius:pos(1)+radius
        for y = pos(2)-radius:pos(2)+radius
            for z = pos(3)-radius:pos(3)
                if round(distance([x,y,z], pos)) == radius
                    demi(i,:) = [x,y,z];
                    i=i+1;
                end
            end
        end
    end
    demi(i:length(demi),:) = [];
end

function b = isFrontier(map, P)
    nbrs = zeros(6,3);
    siz = size(map);
    nbrs(1,:) = [max(P(1)-1,1),P(2),P(3)];
    nbrs(2,:) = [P(1),max(P(2)-1,1),P(3)];
    nbrs(3,:) = [P(1),P(2),max(P(3)-1,1)];
    nbrs(4,:) = [min(P(1)+1,siz(1)),P(2),P(3)];
    nbrs(5,:) = [P(1),min(P(2)+1,siz(2)),P(3)];
    nbrs(6,:) = [P(1),P(2),min(P(3)+1,siz(3))];
    
    unknown = 0;
    free = 0;
    occupied = 0;
    for i = 1:length(nbrs)
        switch map(nbrs(i,1),nbrs(i,2),nbrs(i,3))
            case -1
                unknown = unknown+1;
            case 0
                free = free+1;
            case 1
                occupied = occupied+1;
        end
    end
    b = (map(P(1),P(2),P(3))==0 && unknown>0 && occupied>0);
end
