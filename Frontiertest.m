% 0=libre, 1=occupée -1=inconnu
world = zeros(200,200,200);

%creating environment
cube1 = cube([50,50,80], [150,150,110]);
cube2 = cube([30,30,10], [50,50,120]);
cube3 = cube([70,70,110], [120,120,160]);
cube4 = cube([200,200,200], [180,180,1]);
environment1 = [cube1;cube2;cube3;cube4];
environment2 = sphere([100,100,100],25);

world = insert(world, [environment2], 1);

drone_pos = [60,40,170]; %positione du spectateur
drone_maxDist = 500; %distance maximale à laquelle points sonts detectés
siz = size(world); %pour racourcir du code plus tard
seenWorld = -ones(siz); 
FOV = demisphere(150, drone_pos); %nuage de points pour les raycast, le radius n'a rien a voir avec drone_maxDist

seen = zeros(length(FOV)+1,3); %tous points occupés et detectés par le drone + position du drone
seen(1,:) = drone_pos; 
unseen = zeros(siz(1)*siz(2)*siz(3),3); %touts points occupés et non detectés
front = zeros(round((2/3)*pi*drone_maxDist^3),3); %touts points considérés comme frontière


for index = 1:length(FOV) %raycast de drone_pose à chaque point du demisphere FOV
    rc = raycast(world, drone_pos, [FOV(index,1),FOV(index,2),FOV(index,3)],drone_maxDist);
    for r = 1:length(rc) %touts points passés par le raycast ne sont plus inconnus et donc enregistré dans seenWorld
        seenWorld(rc(r,1), rc(r,2), rc(r,3)) = world(rc(r,1), rc(r,2), rc(r,3));
    end
end

f=1; %compteur de vraie longeur de front
s=2; %compteur de vraie longeur de seen
u=1; %compteur de vraie longeur de unseen

for x = 1:siz(1)
    for y = 1:siz(2)
        for z = 1:siz(3)
            if isFrontier(seenWorld, [x,y,z])
                front(f,:) = [x,y,z];
                f = f+1;
            end
            if seenWorld(x,y,z) == 1
                seen(s,:) = [x,y,z];
                s = s+1;
            elseif world(x,y,z) == 1
                unseen(u,:) = [x,y,z];
                u = u+1;
            end
        end
    end
end

%coupage des array avec leur vraie longeur maintenant connue
seen(s:length(seen),:) = [];
unseen(u:length(unseen),:) = [];
front(f:length(front),:) = [];

%{
%graphique pour touts les points occupés détectés et non détectés
figure;
draw = [seen;unseen];
Length = length(draw);
scale = 5 * ones(Length,1); %largeur des points détectés
scale(1) = 100; %largeur du point du drone
scale(length(seen)+1:Length) = 10; %largeur des points non détectés
color = 0.5 * ones(Length, 3); %couleur des points non détectés
color(1,:) = [1,0,0]; %couleur du point du drone
color(2:length(seen),:) = repmat([0,0,1],length(seen)-1,1); %couleur des points détectés
scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);
%}

%graphique montrant points occupés détectés et points frontièrs
figure;
draw = [seen;front];
Length = length(draw);
scale = 5 * ones(Length,1); %largeur des points détectés
scale(1) = 100; %largeur du point drone
color = repmat([1,0.5,0],Length,1); %couleur des points frontièrs
color(1,:) = [1,0,0]; %couleur du point drone
color(2:length(seen),:) = repmat([0,0,1],length(seen)-1,1); %couleur des points détectés
scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
axis([1,siz(1),1,siz(2),1,siz(3)]);

function points = cube(P1, P2)
%rend un nuage de points formant un cube avec les coins à P1(1x3) et P2(1x3)
%très lent à partir de 10^5 points
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

function points = sphere(pos, radius)
%rend un nuage de points formant une sphere avec les centre à pos(1x3)
    points = zeros(round((4/3)*pi*radius^3),3);
    j = 1;
    for x = pos(1,1)-radius:pos(1,1)+radius
        for y = pos(1,2)-radius:pos(1,2)+radius
            for z = pos(1,3)-radius:pos(1,3)+radius
                if distance(pos, [x,y,z]) <= radius
                    points(j,:) = [x,y,z];
                    j = j+1;
                end
            end    
        end
    end
    points(j:length(points),:) = [];
end

function ins = insert(map, pointcloud, value)
%change la valeur des élements dans map indiqués par les élements de
%pointcloud(~x3) et rend la map changé
    for i = 1:length(pointcloud)
        map(pointcloud(i, 1), pointcloud(i, 2), pointcloud(i, 3)) = value;
    end
    ins = map;
end

function ray = raycast(map, From, Towards, maxDist)
%fait un raycast dans map partant de From(1x3) en direction du point
%Towards(1x3). Tous points libres sont mis dans ray(~x3)
%le raycast se termine ou au bord de la map ou au premier point occupé
%il s'arrete aussi après la distance maxDist du point From
%rend touts points passés jusqu'à l'arret du raycast
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
%distance entre deux points P1(1x3) et P2(1x3)
    dist = sqrt((P1(1)-P2(1))^2 + (P1(2)-P2(2))^2 + (P1(3)-P2(3))^2);
end

function demi = demisphere(radius, pos)
%rend touts points sur la surface d'une demi sphere dessous pos(1x3)
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
%rend 1 si P(1x3) est consideré comme frontière dans map(~x3) et 0 si non
    nbrs = zeros(6,3); %neighbours
    siz = size(map);
    n=1;
    %{
    for x = max(P(1)-1,1):min(P(1)+1,siz(1))
        for y = max(P(2)-1,1):min(P(2)+1,siz(2))
            for z = max(P(3)-1,1):min(P(3)+1,siz(3))
                if P(1) ~= x || P(2) ~= y || P(3) ~= z
                    nbrs(n,:) = [x,y,z];
                    n = n+1;
                end
            end
        end
    end
    nbrs(n:end,:) = [];
    %}
    nbrs(1,:) = [max(P(1)-1,1),P(2),P(3)];
    nbrs(2,:) = [P(1),max(P(2)-1,1),P(3)];
    nbrs(3,:) = [P(1),P(2),max(P(3)-1,1)];
    nbrs(4,:) = [min(P(1)+1,siz(1)),P(2),P(3)];
    nbrs(5,:) = [P(1),min(P(2)+1,siz(2)),P(3)];
    nbrs(6,:) = [P(1),P(2),min(P(3)+1,siz(3))];
    %}
    
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
    %37b = map(P(1),P(2),P(3))==-1 && occupied>0 && free>0;
    %48b = map(P(1),P(2),P(3))==0 && unknown^(1/3)>occupied^(1/2) && unknown>0 && occupied>0;
    %59b = map(P(1),P(2),P(3))==0 && unknown>occupied && unknown>0 && occupied>0;
end
