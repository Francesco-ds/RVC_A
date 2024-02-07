function [time,q,frenet_frames] = simplified_circular_path(start,finish,centre,st)


r = norm(start-centre);
v_start_centre = (start-centre)/r;
v_finish_centre = (finish-centre)/r;
circle_axis = cross(v_start_centre,v_finish_centre);
circle_axis = circle_axis/norm(circle_axis);

theta = acos(dot(v_start_centre,v_finish_centre) / (norm(v_start_centre) * norm(v_finish_centre))); % find angle between 2 vectors: theta = cos-1 [(a·b) / (|a| |v_finish_centre|)]
angles = [0:st:theta]; % angles from start to end

R = [v_start_centre, cross(circle_axis, v_start_centre), circle_axis];
q = centre + R * [r*cos(angles); r*sin(angles); zeros(size(angles))];

%% frenet frames
T = R * [-r .* sin(angles);
    r .* cos(angles);
    zeros(size(angles))];
T = T ./ norm(T);

N = R * [-r .* cos(angles);
    -r .* sin(angles);
    zeros(size(angles));];
N = N ./ norm(N);

B = cross(T, N);
B = B ./ norm(B);

frenet_frames.T = T;
frenet_frames.N = N;
frenet_frames.B = B;

time = angles;


end