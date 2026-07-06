function probes = detectProbes_flyover()
    maxNumCompThreads(4);   % nie zabieraj wszystkich rdzeni TY JEBANA KURWO

    setenv('ROS_DOMAIN_ID', '0');

    d = DroneControllerMATLAB();
    imgSub = ros2subscriber(d.Node, "/rgb_camera/image", "sensor_msgs/Image", ...
                            "Reliability","reliable");

    W=1920; H=1080; hfov=1.5708;
    fx=(W/2)/tan(hfov/2); fy=fx; cx=W/2; cy=H/2;

    scale = 0.5;                       
    fxs=fx*scale; fys=fy*scale;
    intr = cameraIntrinsics([fxs fys],[cx*scale cy*scale],[H*scale W*scale]);

    dict = "DICT_ARUCO_ORIGINAL";      % original library
    markerSize = 0.15;                 % 15 cm

    scanAlt = 3.5;
    d.arm();
    d.takeoff(scanAlt);

    waypointsY = [0, 1.5, -1.5, 0];
    prevY = 0;
    FRAMES_PER_WP = 3;                 % bylo 8

    detections = [];

    for wp = 1:numel(waypointsY)
        dY = waypointsY(wp) - prevY;
        prevY = waypointsY(wp);
        d.goto_relative(0, dY, 0);
        pause(1.5);

        for f = 1:FRAMES_PER_WP
            img = grabLatest(imgSub);          % flush + najnowsza klatka
            if isempty(img), continue; end
            imgS = imresize(img, scale);       % downscale raz, uzywany do CV

            try
                [ids,locs,poses] = readArucoMarker(imgS,dict,intr,markerSize);
            catch
                ids=[];
            end
            if isempty(ids), continue; end
            k101 = find(ids==101,1);
            if isempty(k101), continue; end

            t = poses(k101).Translation;
            h = t(3);
            cM = mean(locs(:,:,k101),1);       % srodek markera w skali downscale

            centers = detectProbePixels(imgS); % piksele w skali downscale

            for i = 1:size(centers,1)
                du = centers(i,1) - cM(1);
                dv = centers(i,2) - cM(2);
                probeZ = 0.10;                     % srodek geom. probe nad ziemia
                hEff   = h - probeZ;               % kamera->plaszczyzna srodka probe
                camX = du * hEff / fxs;
                camY = dv * hEff / fys;
                mE =  camX;                     % <-- dostroj znak na znanym probe
                mN =  camY;
                detections(end+1,:) = [mN, mE]; %#ok<AGROW>
            end
        end
    end

    d.land();

    probes = clusterDetections(detections, 0.25);
    for i = 1:numel(probes)
        probes(i).sector = probeSector(probes(i).E, probes(i).N);
        fprintf("probe %d: N=%.2f E=%.2f -> %s\n", ...
                i, probes(i).N, probes(i).E, probes(i).sector);
    end
end

function img = grabLatest(sub)
    img = [];
    for k = 1:20
        try
            msg = receive(sub, 0.05);   
        catch
            break;                      
        end
        img = rosReadImage(msg);
    end
    if isempty(img)
        try
            msg = receive(sub, 5);      
            img = rosReadImage(msg);
        catch
        end
    end
end

function centers = detectProbePixels(img)
% Zielono-zolty probe w RGB (taniej niz rgb2hsv na kazdej klatce).
    R=img(:,:,1); G=img(:,:,2); B=img(:,:,3);
    mask = G>120 & R>60 & R<220 & B<120 & (G>B+40);
    mask = bwareaopen(mask, 8);          % prog powierzchni w skali downscale
    st = regionprops(mask,'Centroid');
    centers = zeros(numel(st),2);
    for i=1:numel(st), centers(i,:)=st(i).Centroid; end
end

function probes = clusterDetections(det, thr)
    probes = struct('N',{},'E',{},'n',{});
    if isempty(det), return; end
    used = false(size(det,1),1);
    for i=1:size(det,1)
        if used(i), continue; end
        dist = hypot(det(:,1)-det(i,1), det(:,2)-det(i,2));
        grp = dist<thr & ~used;
        used(grp)=true;
        P = det(grp,:);
        probes(end+1) = struct('N',median(P(:,1)),'E',median(P(:,2)),'n',sum(grp)); %#ok<AGROW>
    end
end

function id = probeSector(x, y)
    persistent M
    if isempty(M)
        cells = {
            0  2 'A1';  0  1 'A2';  1  1 'A3';  1  0 'A4';  2  0 'A5';
            2 -1 'B1';  1 -1 'B2';  1 -2 'B3';  0 -2 'B4';  0 -3 'B5';
           -1 -3 'C1'; -1 -2 'C2'; -2 -2 'C3'; -2 -1 'C4'; -3 -1 'C5';
           -3  0 'D1'; -2  0 'D2'; -2  1 'D3'; -1  1 'D4'; -1  2 'D5'};
        M = containers.Map('KeyType','char','ValueType','char');
        for k=1:size(cells,1)
            M(sprintf('%d,%d',cells{k,1},cells{k,2}))=cells{k,3};
        end
    end
    key = sprintf('%d,%d', floor(x/0.5), floor(y/0.5));
    if isKey(M,key), id=M(key); else, id=''; end
end
