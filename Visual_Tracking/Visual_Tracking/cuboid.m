t_x = 3;
t_y = 1;
t_z = 80;
fov = 60;
theta_x = -20  * pi / 180;

x_0 = (300 - 1) / 2;
y_0 = (400 - 1) / 2;
f = (400 / 2) / (tan((fov / 2) * pi /180));

E = [1, 0, 0, t_x; 0, cos(theta_x), -sin(theta_x), t_y; 0, sin(theta_x), cos(theta_x), t_z];
K = [f, 0, x_0; 0, f, y_0; 0, 0, 1];
P = K * E;

v1 = [0; 0; 0; 1];

v = P*v1;