vertices = [0 0 0; 20 0 0; 20 10 0; 0 10 0; 0 0 30; 20 0 30; 20 10 30; 0 10 30];
vertices_hmgns = [0 0 0 1; 20 0 0 1; 20 10 0 1; 0 10 0 1; 0 0 30 1; 20 0 30 1; 20 10 30 1; 0 10 30 1];
vertices_camera = zeros(8, 3);
surfaces =  [2 1 4 3; 1 5 8 4; 5 6 7 8; 6 2 3 7; 8 7 3 4; 5 6 2 1];

% Image dimensions
WIDTH = 400;
HEIGHT = 300;

% Camera Extrinsics
%Camera position
t_x = 1;
t_y = 1;
t_z = 90;
% Rotation x-axis
theta_x = 15 * pi / 180;
% Rotation y-axis
theta_y = 180 * pi / 180;
% Rotation z-axis
theta_z = 180 * pi / 180;
% Camera Intrinsics
% F.O.V
fov = 50;
% Principal point - center of image plane
u_0 = WIDTH / 2;
v_0 = HEIGHT / 2;
% Focal length
f = (WIDTH / 2) / (tan((fov / 2) * pi / 180));
% Camera intrinsics Matrix
K = [f 0 u_0; 0 f v_0; 0 0 1];
% Camera calibration
% Translation Vector
T = [t_x; t_y; t_z];
% Rotation x-axis Matrix
R_x = [1 0 0; 0 cos(theta_x) -sin(theta_x); 0 sin(theta_x) cos(theta_x)];
% Rotation y-axis Matrix
R_y = [cos(theta_y) 0 sin(theta_y); 0 1 0; -sin(theta_y) 0 cos(theta_y)];
% Rotation z-axis Matrix
R_z = [cos(theta_z) -sin(theta_z) 0; sin(theta_z) cos(theta_z) 0; 0 0 1];
% Rotation Matrix
R = R_z * R_y * R_x;
% Camera extrinsics
E = [R(1, 1) R(1, 2) R(1, 3) T(1); R(2, 1) R(2, 2) R(2, 3) T(2); R(3, 1) R(3, 2) R(3, 3) T(3)];
% Perpective Projection Matrix
P = K * E;

for i = 1 : size(vertices_hmgns, 1)
    vertices_camera(i, :) = P * vertices_hmgns(i, :)';
end

occlusions = zeros(size(surfaces, 1), 1);
norm_vec = zeros(size(surfaces, 1), 3);
% Vectors begining from down right vertex of surface
for i = 1 : size(surfaces, 1)
    surface = surfaces(i, :);
    vec1 = vertices_camera(surface(1), :) - vertices_camera(surface(2), :);
	vec2 = vertices_camera(surface(3), :) - vertices_camera(surface(2), :);
	% Normal vector of surface
	vec1 = vec1 / norm(vec1);
    vec2 = vec2 / norm(vec2);
	norm_vec(i, :) = cross(vec1, vec2);
	norm_vec(i, :) = norm_vec(i, :) / norm(norm_vec(i, :));
	% Vector between camera position and down right vertex of surface
	vec_look = T' - vertices_camera(surface(1), :);
	vec_look = vec_look /  norm(vec_look)
	% Angle between look vector and normal of surface
	theta = acos(dot(vec_look, norm_vec(i, :))) * 180 / pi

	% If the angle between them is smaller than 90 degrees then the surface it's visible
	if (theta > 90)
	 occlusions(i) = 1;
    end
end
