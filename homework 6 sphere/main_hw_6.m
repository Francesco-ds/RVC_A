clc;
clear;
st = 0.05;
r = 0.05;

p0 = [0 0 0]';
[X,Y,Z] = sphere(50);
X = X * r + p0(1);
Y = Y * r + p0(2);
Z = Z * r + p0(3);
sphere_points = [X(:) Y(:) Z(:)];
points = [];

%take 3 random points
for i = 1:1:3
    points = [points sphere_points(randi(size(sphere_points,1)),:)'];
    
end

q = [];
frenet_T = [];
frenet_N = [];
frenet_B = [];

for i = 1:2

    start = points(:,i);
    finish = points(:,i+1);
    [q_add,frenet_frames] = circular_path_frenet(start,finish,p0,st,1);
    q = [q q_add];
    frenet_T = [frenet_T frenet_frames.T];
    frenet_N = [frenet_N frenet_frames.N];
    frenet_B = [frenet_B frenet_frames.B];
end

%% plot sphere, points, q_add
h = surf(X, Y, Z, 'FaceColor', [0.68, 0.85, 0.9], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
axis equal
hold on;
scatter3(points(1,:), points(2,:), points(3,:), 60, 'filled', 'r');
plot3(q(1,:),q(2,:),q(3,:),'LineWidth',2,'color','g');

%quiver3(q(1,:),q(2,:),q(3,:), frenet_T(1,:),frenet_T(2,:),frenet_T(3,:),'Color','red');
quiver3(q(1,:),q(2,:),q(3,:), -frenet_N(1,:),-frenet_N(2,:),-frenet_N(3,:),'Color','green');
quiver3(q(1,:),q(2,:),q(3,:), frenet_B(1,:),frenet_B(2,:),frenet_B(3,:),'Color','blue');
