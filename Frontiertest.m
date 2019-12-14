maxAlgos = 11;
maxEnvs = 4;


allEvaluations = zeros(maxAlgos, 5, maxEnvs, 4);
filtering = 0;
perfectEnv = 0;
lastEnv = 0;
FrontierEvaluation = zeros(maxAlgos, 5, maxEnvs);
resolution = 2*[360, 90]; %nombre de raycast faits [horizontal, vertical]

for env=1:4
    
    drone_maxDist = 200; %distance maximale à laquelle points sonts detectés
    drone_hor_view = [0, 360]; %angle de vue du drone à l'horizontal [de, à] partant de l'axis X (sens mathématique)
    drone_ver_view = [0, -90]; %angle de vue du drone au vertical [de, à] partant de l'axis X (sens mathématique)
    %FlightPath = [[(10:6:160)',(70:-2:20)',(180:-4:80)'];[(160:-1:140)',(20:8:180)',(80:6:200)']];
    FlightPath = [100,100,125];
    
    disp(strcat('Preparing Environment',num2str(env),'...'));
    %creating environment
    switch env
        case 1 %perfect frontier: 400
            FlightPath = [100,100,125];
            drone_maxDist = 200;
            block = cube([50,50,100], [150,150,100]);
            
            unk = cube([49,49,99],[151,151,99]);
            air2 = cube([1,1,1],[2,2,2]);
            mustPoints = [line([49,49,100],[151,49,100]);
                          line([151,50,100],[151,151,100]);
                          line([150,151,100],[49,151,100]);
                          line([49,150,100],[49,50,100])];
            air = [mustPoints;cube([49,49,100], [151,151,125])];          
            
        case 2 %perfect frontier: 60 * Pi = 188.5
            FlightPath = [100,100,125];
            drone_maxDist = round(norm([24,29]));
            block = [100,100,100];
            for r = 30:-0.5:1
                block = [block;circle([100,100,100],r)];
            end
            unk = cube([70,70,70],[130,130,99]);
            air2 = [];
            mustPoints = circle([100,100,101], 30);
            air = [mustPoints;sphere([100,100,101], 30)];
            
        case 3 %perfect frontier: 100 * Pi = 314
            drone_maxDist = norm([89,19]);
            FlightPath = [100,100,150];
            for phi = 0:0.125:1.875
                FlightPath = [FlightPath;[100+70*cos(pi*phi),100+70*sin(pi*phi), 150]];
            end
            block = sphere([100,100,60], 50);
            mustPoints = circle([100,100,60],50);
            air = [mustPoints;cube([49,49,60], [151,151,120])];
            unk = [sphere([100,100,60], 49);cube([50,50,10],[150,150,59])];
            air2 = sphere([100,100,60], 45);
            
        case 4
            drone_maxDist = 200;
            FlightPath = [100,100,180];
            mustPoints = [line([49,50,50], [49, 150, 50]);line([151,50,150], [151, 150, 150])];
            mustPoints = [mustPoints;line([50,49,50],[150,49,150,]);line([50,151,50],[150,151,150,])];
            air = [mustPoints;line([50,49,50],[150,49,150,]);line([50,151,50],[150,151,150,])];
            air = [air;line([50,49,51],[150,49,151,]);line([50,151,51],[150,151,151,])];
            block = [];
            for x = 50:150
                block = [block;line([x,50,x],[x,150,x])];
                air = [air;cube([x-1,48,x],[x-1,152,min(x+100,200)])];
            end
            air2 = [];
            unk = [];
            air = [air;cube([x,48,x+1],[x,152,200])];
            air = [air;cube([x+1,48,x],[x+1,152,200]);line([150,49,151],[150,151,151])];  
            
    end
    
    % 0=libre, 1=occupée -1=inconnu
    FPlength = length(FlightPath(:,1));
    world = zeros(200,200,200);
    siz = size(world); %pour racourcir du code plus tard
    seenWorld = -ones(siz);
    world = insert(world, block, 1);
    world = insert(world, air2, 0);
    
    if perfectEnv == 1
            seenWorld = insert(seenWorld, air, 0);
            seenWorld = insert(seenWorld, block, 1);
            seenWorld = insert(seenWorld, unk, -1);
    else
        for posCounter = 1:FPlength
            drone_pos = FlightPath(posCounter,:);
            phi_step = (drone_hor_view(2) - drone_hor_view(1))/resolution(1);
            theta_step = (drone_ver_view(2) - drone_ver_view(1))/resolution(2);
            for phi = drone_hor_view(1):phi_step:drone_hor_view(2)
                for theta = drone_ver_view(1):theta_step:drone_ver_view(2)
                    rc = raycast(world, drone_pos, phi, theta, drone_maxDist);
                    for r = 1:length(rc(:,1)) %touts points passés par le raycast ne sont plus inconnus et donc enregistré dans seenWorld
                        seenWorld(rc(r,1), rc(r,2), rc(r,3)) = world(rc(r,1), rc(r,2), rc(r,3));
                    end
                end
            end
        end
    end
    
    disp('Performing EdgeDetection...');
    avgEdgesDetected = EdgeDetect(seenWorld);
    edgesDetected = EdgeDetect(seenWorld);
    edgeThreshold = [0.7,1];
    
    for algo=1:11
        disp(strcat('Now working on: Environment',num2str(env),',','Algorythm',num2str(algo)));

        alledges = zeros(200^3, 3);
        seen = zeros(resolution(1)*resolution(2)+FPlength,3); %tous points occupés et detectés par le drone + position du drone
        seen(1:FPlength,:) = FlightPath; 
        unseen = zeros(siz(1)*siz(2)*siz(3),3); %touts points occupés et non detectés
        front = zeros(round((2/3)*pi*drone_maxDist^3),3); %touts points considérés comme frontière
    
        e = 1;
        f=1; %compteur de vraie longeur de front
        s=FPlength+1; %compteur de vraie longeur de seen
        u=1; %compteur de vraie longeur de unseen
        tp=1;
        for x = 1:siz(1)
            for y = 1:siz(2)
                for z = 1:siz(3)
                    if edgesDetected(x,y,z) >= edgeThreshold(1) && edgesDetected(x,y,z) <= edgeThreshold(2)
                        alledges(e,:) = [x,y,z];
                        e = e + 1;
                    end
                    if seenWorld(x,y,z) == 1
                        seen(s,:) = [x,y,z];
                        s = s+1;
                    elseif seenWorld(x,y,z) == 0
                        if isFrontier(seenWorld, [x,y,z], algo, edgesDetected, edgeThreshold)
                            front(f,:) = [x,y,z];
                            f = f+1;
                        end
                    elseif world(x,y,z) == 1
    
                        unseen(u,:) = [x,y,z];
                        u = u+1;
                    end
                end
            end
        end
    
        %coupage des array avec leur vraie longeur maintenant connue
        seen(s:end,:) = [];
        unseen(u:end,:) = [];
        front(f:end,:) = [];
        alledges(e:end,:) = [];
        
        if perfectEnv == 0 && filtering == 1
            disp('Deleting small groups of fronts');
            filterSmallGroups(front, 5);
        end
        
        disp('Evaluating...');
        primaryFront = comparePointClouds(front, mustPoints,1);
        nonPrimaryFront = comparePointClouds(front, mustPoints,0);
        secondaryFront = nonPrimaryFront;
        s =1;
        %if perfectEnv == 0
            for i = 1:length(nonPrimaryFront(:,1))
                shortestDistance = inf;
                for j = 1:length(mustPoints(:,1))
                    shortestDistance = min(shortestDistance, distance(nonPrimaryFront(i,:),mustPoints(j,:))); 
                end
                if shortestDistance <= 5
                    secondaryFront(s,:) = nonPrimaryFront(i,:);
                    s = s+1;
                end
            end
        %{
        else
            for i=1:length(nonPrimaryFront(:,1))
                closestIndex = 0;
                dist = inf;
                for j=1:length(primaryFront(:,1))
                    curDist = distance(primaryFront(j,:), nonPrimaryFront(i,:));
                    if curDist < dist && curDist < 10
                        dist = curDist;
                        closestIndex = j;
                    end
                end
                if closestIndex == 0
                    %do nothing, this is here, so "ConnectionTo" is not
                    %called when not needed
                elseif ConnectionTo(front, nonPrimaryFront(i,:), mustPoints(closestIndex,:), 5, inf) > -1
                    secondaryFront(s,:) = nonPrimaryFront(i,:);
                    s = s+1; 
                end
                
            end
        end
        %}
        secondaryFront(s:end,:) = [];
        
        FrontierEvaluation(algo,1,env) = 100 * length(primaryFront(:,1))/length(mustPoints(:,1));
        FrontierEvaluation(algo,2,env) = 100 * length(primaryFront(:,1))/length(front(:,1));
        FrontierEvaluation(algo,3,env) = 100 * length(secondaryFront(:,1))/length(front(:,1));
        FrontierEvaluation(algo,4,env) = 100-(FrontierEvaluation(algo,2,env)+FrontierEvaluation(algo,3,env));
        FrontierEvaluation(algo,end,env) = length(front(:,1))/1000;
    
        if lastEnv < env
            lastEnv = env;
            if length(alledges) <= 100000
            %graphic for edge detection
                if perfectEnv == 1
                    title = strcat('EdgeDetection in PerfectEnvironment Nr.' , num2str(env));
                else
                    title = strcat('EdgeDetection in RealEnvironment Nr.' , num2str(env), '_Res[',num2str(resolution(1)),',',num2str(resolution(2)),']');
                end
                figure('name',title);
                color = zeros(length(alledges(:,1)),3);
                for i = 1:length(alledges(:,1))
                    color(i,:) = 1 - edgesDetected(alledges(i,1),alledges(i,2), alledges(i,3));
                end
                scatter3(alledges(:,1),alledges(:,2),alledges(:,3),5,color,'filled','o');
                axis([1,siz(1),1,siz(2),1,siz(3)]);
                saveas(gcf,[pwd strcat('\generatedFigures\',title,'.fig')]);
                set(gcf, 'Visible', 'off');
            end
    
            %graphique pour touts les points occupés détectés et non détectés
            if perfectEnv == 1
                title = strcat('Overview in PerfectEnvironment Nr.' , num2str(env));
            else
                title = strcat('Overview in RealEnvironment Nr.' , num2str(env), '_Res[',num2str(resolution(1)),',',num2str(resolution(2)),']');
            end
            figure('name',title);
            draw = [seen;unseen;mustPoints];
            Length = length(draw(:,1));
            scale = 5 * ones(Length,1); %largeur des points détectés
            scale(1:FPlength) = 50; %largeur du point drone
            scale(length(seen(:,1))+1:Length) = 5; %largeur des points non détectés
            color = 0.5 * ones(Length, 3); %couleur des points non détectés
            color(1:FPlength,:) = repmat([1,0,0],FPlength,1); %couleur du point drone
            color(1+FPlength:length(seen(:,1)),:) = repmat([0,0,1],length(seen(:,1))-FPlength,1); %couleur des points détectés
            color(length(draw(:,1))-length(mustPoints(:,1)):end,:) = repmat([0,0.5,0],length(mustPoints(:,1))+1,1);
            scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
            axis([1,siz(1),1,siz(2),1,siz(3)]);
            saveas(gcf,[pwd strcat('\generatedFigures\',title,'.fig')]);
            set(gcf, 'Visible', 'off');
        end
    
    
        %graphique montrant points occupés détectés et points frontièrs
        if perfectEnv == 1
            title = strcat('Frontiers in PerfEnv Nr.' , num2str(env) , ' Algo Nr.' , num2str(algo));
        else
            title = strcat('Frontiers in RealEnv Nr.' , num2str(env) , ' Algo Nr.' , num2str(algo), '_Res[',num2str(resolution(1)),',',num2str(resolution(2)),']');
        end
        figure('name',title);
        draw = [seen;front];
        Length = length(draw(:,1));
        scale = 5 * ones(Length,1); %largeur des points détectés et frontières
        scale(1:FPlength) = 50; %largeur du point drone
        color = repmat([1,0.5,0],Length,1); %couleur des points frontièrs
        color(1:FPlength,:) = repmat([1,0,0],FPlength,1); %couleur du point drone
        color(1+FPlength:length(seen(:,1)),:) = repmat([0,0,1],length(seen(:,1))-FPlength,1); %couleur des points détectés
        scatter3(draw(:,1),draw(:,2),draw(:,3),scale,color,'filled','o');
        axis([1,siz(1),1,siz(2),1,siz(3)]);
        saveas(gcf,[pwd strcat('\generatedFigures\',title,'.fig')]);
        set(gcf, 'Visible', 'off');
    end
end
disp(strcat('Res:',num2str(resolution),'_PerfectEnvironment:',num2str(perfectEnv)));
disp(FrontierEvaluation);

    





function points = comparePointClouds(pCloud1, pCloud2, shared)
%shared == 1: returns all shared points
%shared == 0: returns is pCloud1 without pCloud2
    points = pCloud1;
    p = 1;
    for i=1:length(pCloud1(:,1))
        if pointCloudContains(pCloud2,pCloud1(i,:)) == shared
            points(p,:) = pCloud1(i,:);
            p = p+1;
        end
    end
    points(p:end,:) = [];
end

function b = pointCloudContains(pCloud, P)
    b = 0;
    for i = 1:length(pCloud(:,1))
        if min(pCloud(i,:) == P) == 1
            b = 1;
            return;
        end
    end
end

function points = line(P1, P2)
    points = zeros(1+round(norm([P1(1)-P2(1),P1(2)-P2(2),P1(3)-P2(3)])),3);
    Dir = (P2 - P1)/max(abs(P2-P1));
    p = 1;
    for i = 0:max(abs(P2-P1))
        points(i+1,:) = round(P1 + i * Dir);
        p = p+1;
    end
    points(p:end,:) = [];
end

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

function points = circle(pos, radius)
    points = zeros(round(2*pi*radius),3);
            i = 1;
            for x = pos(1)-radius-1:pos(1)+radius+1
                for y = pos(2)-radius-1:pos(2)+radius+1
                    if abs(distance(round([x,y,pos(3)]), pos) - radius) <= 0.5
                        points(i,:) = round([x,y,pos(3)]);
                        i = i+1;
                    end
                end
            end
    points(i:end,:) = [];
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
    siz = size(map);
    for i = 1:length(pointcloud)
        if min(pointcloud(i,:)) > 0 && pointcloud(i, 1) <= siz(1) && pointcloud(i, 2) <= siz(2) && pointcloud(i, 3) <= siz(3)
            map(pointcloud(i, 1), pointcloud(i, 2), pointcloud(i, 3)) = value;
        end
    end
    ins = map;
end

function ray = raycast(map, From, phi, theta, maxDist)
%fait un raycast dans map partant de From(1x3) en direction du point
%Towards(1x3). Tous points libres sont mis dans ray(~x3)
%le raycast se termine ou au bord de la map ou au premier point occupé
%il s'arrete aussi après la distance maxDist du point From
%rend touts points passés jusqu'à l'arret du raycast
    Dir = [cos(pi*phi/180)*cos(pi*theta/180), sin(pi*phi/180)*cos(pi*theta/180), sin(pi*theta/180)];
    ray = zeros(round(1.5*maxDist), 3);
    i = 1;
    Next = From+Dir;
    limit = size(map);
    if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
        ray(i,:) = round(Next);
        ray(i+1:end,:) = [];
        return;
    end
    while not(distance(Next, From) >= maxDist || map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
        i = i+1;
        Next = Next + Dir;
        if round(Next(1)) > limit(1) || round(Next(2)) > limit(2) || round(Next(3)) > limit(3) || round(min(Next)) < 1
            ray(i:end,:) = [];
            return;
        end
    end
    if (map(round(Next(1)),round(Next(2)),round(Next(3))) == 1)
        ray(i,:) = round(Next);
        ray(i+1:end,:) = [];
    else
        ray(i:end,:) = [];
    end
end

function dist = distance(P1, P2)
%distance entre deux points P1(1x3) et P2(1x3)
    dist = norm([P1(1)-P2(1), P1(2)-P2(2), P1(3)-P2(3)]);
end

function nbrs = getNeighbours(map, P, value, v)
    nbrs = zeros(26,3);
    n=1;
    c = 3; %neighbour cube size 
    siz = size(map);
    for x = max(P(1)-(c-1)/2,1):min(P(1)+(c-1)/2,siz(1))
        for y = max(P(2)-(c-1)/2,1):min(P(2)+(c-1)/2,siz(2))
            for z = max(P(3)-(c-1)/2,1):min(P(3)+(c-1)/2,siz(3))
                if P(1) ~= x || P(2) ~= y || P(3) ~= z
                    Dist = distance([x,y,z],P);
                    
                    if Dist < 2 %voisinage <= 26
                        if v == 26 && map(x,y,z)==value 
                            nbrs(n,:) = [x,y,z];
                            n=n+1;
                        end
                        
                        if Dist < sqrt(3) %voisinage <= 18
                           if v == 18 && map(x,y,z)==value
                               nbrs(n,:) = [x,y,z];
                               n=n+1;
                           end                        
                           
                             if Dist < sqrt(2) %voisinage 6
                                if v == 6 && map(x,y,z)==value
                                    nbrs(n,:) = [x,y,z];
                                    n=n+1;
                                end
                             end
                        end
                    end
                end
            end
        end
    end
    nbrs(n:end,:) = [];
end

function b = isFrontier(map, P, algo,  edges, edgeThreshold)
%rend 1 si P(1x3) est consideré comme frontière dans map(~x3) et 0 si non
    
    switch algo
        case 1
            voi_occ = 6;
            voi_unk = 6;
        case 2
            voi_occ = 18;
            voi_unk = 18;
        case 3
            voi_occ = 26;
            voi_unk = 26;
        case 4
            voi_occ = 26;
            voi_unk = 6;
        case 5 
            voi_occ = 6;
            voi_unk = 26;
        case 6
            voi_occ = 18;
            voi_unk = 6;
        case 7
            voi_occ = 6;
            voi_unk = 18;
        case 8
            voi_occ = 26;
            voi_unk = 6;
        case 9
            voi_occ = 6;
            voi_unk = 26;
        case 10 
            voi_occ = 18;
            voi_unk = 6;
        case 11
            voi_occ = 6;
            voi_unk = 18;
    end
    
    occupied = getNeighbours(map, P, 1, voi_occ);
    unknown = getNeighbours(map, P, -1, voi_unk);
    occupied = length(occupied(:,1));
    unknown = length(unknown(:,1));
   
    if algo <= 7
        b = unknown>0 && occupied>0;
    end
    if algo >= 8
        siz = size(edges);
        nbrs = edges(max(P(1)-1,1):min(P(1)+1,siz(1)), max(P(2)-1,1):min(P(2)+1,siz(2)), max(P(3)-1,1):min(P(3)+1,siz(3)));
        %isEdge = avgEdges(P(1),P(2),P(3)) >= edgeThreshold(3) && avgEdges(P(1),P(2),P(3)) <= edgeThreshold(4);
        %isEdge = isEdge || (Edges(P(1),P(2),P(3)) >= edgeThreshold(1) && Edges(P(1),P(2),P(3)) <= edgeThreshold(2));
        b = max(max(max(nbrs))) >= edgeThreshold(1) && max(max(max(nbrs))) <= edgeThreshold(2) && unknown>0 && occupied>0;
    end
end

function b = ConnectionTo(map, point, goal, maxSteps, value)
%returns -1 if not connected and else the number of steps required
    b = -1;
    if (min(point == goal) == 1)
        b = 0;
        return;
    end
    if value == inf %map is a pointCloud with only relevant points
        if not(pointCloudContains(map, point) && pointCloudContains(map, goal))
            return;
        else
            curPoint = point;
            b = 1;
            while b <= maxSteps
                pointsFound = 0;
                for i=1:length(map(:,1))
                    
                    if distance(goal, map(i,:)) < distance(goal, curPoint) && distance(curPoint, map(i,:)) < 2
                        curPoint = map(i,:);
                        pointsFound = pointsFound+1;
                    end
                end
                    if pointsFound > 0
                        if min(curPoint==goal)==1
                            return;
                        else
                            b = b+1;
                        end
                    else
                        b = -1; 
                        return;
                    end
               
            end
        end
    else %map i a map and only points where map(x,y,z)==value are relevant
        siz = size(map);
        if max(point(1),goal(1)) > siz(1) || max(point(2),goal(2)) > siz(2) || max(point(3),goal(3)) > siz(3) || min(point,goal) < 1
            return;
        else
            curPoint = point;
            b = 0;
            while b <= maxSteps
                
                b=b+1;
                if min(curPoint==goal) == 1
                    return;
                end
                
                nbrs = getNeighbours(map, curPoint, value, 26);
                if not(ISEMPTY(nbrs(:,1)))
                    closestDist = distance(curPoint, goal);
                    for i=1:length(nbrs(:,1))
                        if distance(nbrs(i,:),goal) < closestDist
                            curPoint = nbrs(i,:);
                        end
                    end
                    if closestDist == distance(curPoint, goal)
                        b = -1;
                        return;
                    else
                        
                    end
                else
                    b=-1;
                    return;
                end
            end
        end
    end
    b = -1;
end

function edges = EdgeDetect(map)
    %map((map == val1 | map == val2)) = val1;
    conv_matrix = -ones(3,3,3);
    conv_matrix(2,2,2) = 26;
    edges = convn(map,conv_matrix);
    edges(:,:,:) = edges - min(min(min(edges)));
    edges(:,:,:) = edges / max(max(max(edges)));
    edges(:,:,1) = [];
    edges(:,1,:) = [];
    edges(1,:,:) = [];
    edges(:,:,end) = [];
    edges(:,end,:) = [];
    edges(end,:,:) = [];
end

function filtered = filterSmallGroups(pCloud, minSize)
    filtered = pCloud;
    f = 1;
    for i = 1:length(pCloud(:,1))
        longestConnection = 0;
        for j = 1:length(pCloud(:,1))
            longestConnection = max(ConnectionTo(pCloud, pCloud(i,:), pCloud(j,:), minSize,inf),longestConnection);
        end
        if longestConnection >= minSize
            filtered(f,:) = pCloud(i,:);
            f=f+1;
        end
    end
    filtered(f:end,:) = [];
end

%{
environment parfait vs raycast
raycast round
visualization
edge detection?
%}
