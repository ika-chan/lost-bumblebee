close all; dbstop error; clc;

R = [7.027555e-03 -9.999753e-01 2.599616e-05;-2.254837e-03 -4.184312e-05 -9.999975e-01;9.999728e-01 7.027479e-03 -2.255075e-03];
T = [-7.137748e-03 -7.482656e-02 -3.336324e-01]';
Tr_velo_to_cam = [R,T;[0 0 0 1]];
R_rect = [9.999280e-01 8.085985e-03 -8.866797e-03;-8.123205e-03 9.999583e-01 -4.169750e-03;8.832711e-03 4.241477e-03 9.999520e-01];
P_rect = [7.070912e+02 0.000000e+00 6.018873e+02 4.688783e+01;0.000000e+00 7.070912e+02 1.831104e+02 1.178601e-01;0.000000e+00 0.000000e+00 1.000000e+00 6.203223e-03];

R_cam_to_rect = eye(4);
R_cam_to_rect(1:3,1:3) = R_rect;
P_velo_to_img = P_rect*R_cam_to_rect*Tr_velo_to_cam;

% img = imread(sprintf('%s/%010d.png',IMAGE_DIR,image_num));
% img = rgb2gray(img);
% fig = figure('Position',[20 100 size(img,2) size(img,1)]); axes('Position',[0 0 1 1]);
% imshow(img); hold on;
% nb = [];
table = [133 134 135 136 137 174 176 531 533 546 550 551 553 582 675 676 677 678 679 680 681 682 764 766 767 770 771 772 773 778 783 784 789 790 792 793 794 795 796 901 902 1019 1098 1125 1130 1173 1174 1175 1184 1185 1192 1204 1206 1207 1208 1209 1210 1211 1212 1213 1427 1428 1429 1430 1431 1434 1501 1502 1503 1506 1509 1510 1511 1512 1513 1521 1523 1530 1533 1536 1543 1544 1545 1546 1547 1548 1549 1550 1551 1552 1553 1554 1555 1556 1557 1558 1559 1560 1561 1562 1563 1564 1565 1566 1567 1568 1569 1570 1571 1572 1574 1583 1584 1598 1599 1601 1602 1603 1604 1605 1606 1607 1608 1609 1610 1611 1612 1613 1614 1617 1618 1619 1621 1622 1623 1624 1625 1631 1635 1643 1645 1646 1647 1648 1650 1651 1653 1668 1670 1671 1673 1679 1999 2028 2029 2030 2031 2032 2046 2047 2048 2049 2050 2080 2081 2082 2083 2084 2085 2086 2087 2090 2288 2291 2292 2295 2296 2298 2299 2307 2311 2312 2313 2314 2315 2316 2317 2318 2320 2321 2325 2327 2328 2329 2332 2333 2338 2340 2341 2342 2343 2362 2365 2367 2370 2371 2374 2375 2376 2384];
for image_num=0:199
    landmark = [];
    image_num
    count = 0;
    for i=1:size(table,2)
        if image_num == table(i)
            count = count+1;
        end
    end
    if count==0
        landmarks = csvread(['../Mask_RCNN/output_data/',num2str(image_num),'.csv']);
        XYZ = [];
        for i=1:size(landmarks,1)
            u = mean(landmarks(i,2),landmarks(i,4));
            v = mean(landmarks(i,1),landmarks(i,3));
            XYZ = get_XYZ(image_num,P_velo_to_img,u,v);
            XYZ(1) = XYZ(1)-5;
            if XYZ(1)<=65
                landmark = [landmark;[landmarks(i,:),XYZ]];
            end
        end
    else
        landmark = [];
    end
    csvwrite(['output_landmarks/landmark',num2str(image_num),'.csv'],landmark);
end

function XYZ = get_XYZ2(image_num,u,v)
depth_dir = '2011_09_30_drive_0018_sync/velodyne_points/data';
I = imread(sprintf('%s/%010d.png',depth_dir,image_num));
fx = 7.070912e+02;  % focal length x
fy = 7.070912e+02;  % focal length y
cx = 6.018873e+02;  % optical center x
cy = 1.831104e+02;  % optical center y
depth = double(I)/256;
depth(I==0) = -1;
count = 100;
for x=-1000:1000
    for y=-1000:1000
        if (x+u)<=0
            cor_x = 1;
        elseif (x+u)>size(depth,2)
            cor_x = size(depth,2);
        else
            cor_x = x+u;
        end
        if (y+v)<=0
            cor_y = 1;
        elseif (y+v)>size(depth,1)
            cor_y = size(depth,1);
        else
            cor_y = y+v;
        end
        if (depth(cor_y,cor_x)~=-1) && (count>abs(x)+abs(y))
            m = x+u;
            n = y+v;
            count = abs(x)+abs(y);
        end
    end
end
Z = depth(n,m);
X = (u - cx) * Z / fx;
Y = (v - cy) * Z / fy;
XYZ = [X,Y,Z];
end

function XYZ = get_XYZ(image_num,P_velo_to_img,u,v)
% load velodyne points
LIDAR_DIR = '2011_09_30_drive_0018_sync/velodyne_points/data';
fid = fopen(sprintf('%s/%010d.bin',LIDAR_DIR,image_num),'rb');
velo = fread(fid,[4 inf],'single')';
fclose(fid);

idx = velo(:,1)<5;
velo(idx,:) = [];
velo_img = project(velo(:,1:3),P_velo_to_img);

dist = inf;
n = [];
for i=1:size(velo_img,1)
    if norm(velo_img(i,1:2)-[u,v])<dist
        dist = norm(velo_img(i,1:2)-[u,v]);
        n = [n;i];
    end
end
if size(n,1)>10
    XYZ = mean(velo(n(end-10:end),1:3),1);
else
    XYZ = mean(velo(n,1:3),1);
end
end

function p_out = project(p_in,T)
% dimension of data and projection matrix
dim_norm = size(T,1);
dim_proj = size(T,2);
% do transformation in homogenuous coordinates
p2_in = p_in;
if size(p2_in,2)<dim_proj
  p2_in(:,dim_proj) = 1;
end
p2_out = (T*p2_in')';
% normalize homogeneous coordinates:
p_out = p2_out(:,1:dim_norm-1)./(p2_out(:,dim_norm)*ones(1,dim_norm-1));
end

