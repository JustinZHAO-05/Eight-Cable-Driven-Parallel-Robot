clc;
clear;

% Initialize the dimensions of the container (a cube) and the rectangular box
container_size = 45; % Side length of the cubic container
length = 8; % Length of the rectangular box
width = 10;  % Width of the rectangular box
height = 6; % Height of the rectangular box

% Define the vertices of the rectangular box
vertices = [
    -length/2 -width/2 -height/2;   % Vertex 1
    length/2 -width/2 -height/2;   % Vertex 2
    length/2 width/2 -height/2;   % Vertex 3
    -length/2 width/2 -height/2;   % Vertex 4
    -length/2 -width/2 height/2;   % Vertex 5
    length/2 -width/2 height/2;   % Vertex 6
    length/2 width/2 height/2;   % Vertex 7
    -length/2 width/2 height/2   % Vertex 8
];

% Define the faces of the rectangular box using vertex indices
faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

% Create a figure and set up the 3D view
figure;
axis equal;
xlim([-container_size/2 container_size/2]);
ylim([-container_size/2 container_size/2]);
zlim([-container_size/2 container_size/2]);
grid on;
view(3);

% Define the vertices of the cubic container
container_vertices = [
    -container_size/2, -container_size/2, -container_size/2;
    container_size/2, -container_size/2, -container_size/2;
    container_size/2, container_size/2, -container_size/2;
    -container_size/2, container_size/2, -container_size/2;
    -container_size/2, -container_size/2, container_size/2;
    container_size/2, -container_size/2, container_size/2;
    container_size/2, container_size/2, container_size/2;
    -container_size/2, container_size/2, container_size/2;
];

% Define the faces of the cubic container
container_faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

% Draw the transparent cubic container
hold on;
patch('Faces', container_faces, 'Vertices', container_vertices, 'FaceColor', 'g', 'FaceAlpha', 0.1);

% Initialize position and rotation angles of the rectangular box
position = [0, 0, 0]; % Initial position
angles = [0, 0, 0]; % Initial rotation angles around x, y, z axes

% Define the rotation matrix function
rotation_matrix = @(angle, axis) ...
    [cos(angle) + axis(1)^2 * (1 - cos(angle)), axis(1)*axis(2)*(1 - cos(angle)) - axis(3)*sin(angle), axis(1)*axis(3)*(1 - cos(angle)) + axis(2)*sin(angle);
     axis(2)*axis(1)*(1 - cos(angle)) + axis(3)*sin(angle), cos(angle) + axis(2)^2 * (1 - cos(angle)), axis(2)*axis(3)*(1 - cos(angle)) - axis(1)*sin(angle);
     axis(3)*axis(1)*(1 - cos(angle)) - axis(2)*sin(angle), axis(3)*axis(2)*(1 - cos(angle)) + axis(1)*sin(angle), cos(angle) + axis(3)^2 * (1 - cos(angle))];
 
rotated_vertices = vertices;

% Create the rectangular box patch object
h = patch('Faces', faces, 'Vertices', vertices, 'FaceColor', 'b');

% Initialize red lines connecting the rectangular box to the container vertices
h_line = gobjects(1, 8);  % Preallocate 8 line objects
for i = 1:8
    h_line(i) = plot3([container_vertices(i,1), rotated_vertices(i,1)], ...
                      [container_vertices(i,2), rotated_vertices(i,2)], ...
                      [container_vertices(i,3), rotated_vertices(i,3)], 'Color', 'r', 'LineWidth', 2);
end

% Animation loop
num_frames = 1000;
for k = 1:num_frames
    if k < 200
        position = [0, 0, 0];
        angles = [sin((k-100)*0.01)*pi/3, 0, 0];
    elseif k > 200 && k < 400
        position = [0, 0, 0];
        angles = [0, sin((k-300)*0.01)*pi/3, 0];
    elseif k > 400 && k < 600
        position = [0, 0, 0];
        angles = [0, 0, sin((k-500)*0.01)*pi/3];
    elseif k > 600 && k < 800
        position = [sin(k*0.05) * container_size / 5, cos(k*0.05) * container_size / 5, 0];
        angles = [0, 0, 0];
    elseif k > 800 && k < 1000
        position = [0, 0, sin(k*0.05) * container_size / 10];
        angles = [0, 0, 0];
    end

    % Compute the composite rotation matrix
    R_x = rotation_matrix(angles(1), [1, 0, 0]);
    R_y = rotation_matrix(angles(2), [0, 1, 0]);
    R_z = rotation_matrix(angles(3), [0, 0, 1]);
    R = R_x * R_y * R_z;
    
    % Apply rotation and translation to the rectangular box
    rotated_vertices = (R * vertices')';
    rotated_vertices = rotated_vertices + position;
    
    % Update the patch object with the new vertices
    h.Vertices = rotated_vertices;
    
    % Update the red connecting lines
    for i = 1:8
        set(h_line(i), 'XData', [container_vertices(i,1), rotated_vertices(i,1)], ...
                        'YData', [container_vertices(i,2), rotated_vertices(i,2)], ...
                        'ZData', [container_vertices(i,3), rotated_vertices(i,3)]);
    end
    
    % Force MATLAB to refresh the figure
    drawnow;
end
