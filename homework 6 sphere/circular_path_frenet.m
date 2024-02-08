function [p,frenet_frames] = circular_path_frenet(p1,p2,centre,st,orientation_for_centre_case)


    vector_point1_centre = p1 - centre;
    vector_point2_centre = p2 - centre;
    radius_value = norm(p2-centre);
    

    %z axis of the circumnference cut by the p1 p2 plane
    axis_of_cut = cross(vector_point2_centre,vector_point1_centre);
    versor_1 = vector_point1_centre/norm(vector_point1_centre);
    versor_2 =vector_point2_centre/norm(vector_point2_centre);
    angle_point1_point2 = acos(dot(versor_1,versor_2));

    %if the cross product is zero then i need to force one direction, to do
    %this i modify the value v2 in such a way to get the cross product on
    %the wanted direction

    if (axis_of_cut == 0)
        % i check which values i can use to modify(must be different from zero!!!)
        if(vector_point1_centre(1) ~= 0)
            vector_point2_centre = [vector_point1_centre(2);vector_point1_centre(1);vector_point1_centre(3)];
        elseif(vector_point1_centre(2) ~= 0)
            vector_point2_centre = [vector_point1_centre(1);-vector_point1_centre(3);vector_point1_centre(2)];
        elseif(vector_point1_centre(3) ~= 0)
             vector_point2_centre = [vector_point1_centre(1);vector_point1_centre(3);vector_point1_centre(2)];
        end

        axis_of_cut = cross(vector_point2_centre,vector_point1_centre) * orientation_for_centre_case; %decide the direction in which i want the z axis

      
    end 

    x_axis_circ = (p1-centre)/ norm(p1-centre);
    z_axis_circ = axis_of_cut/ norm(axis_of_cut);
    y_axis_circ = cross(x_axis_circ,z_axis_circ);

    R = [x_axis_circ,y_axis_circ,z_axis_circ];
    s = 0:st:angle_point1_point2;
    p = centre + R * [radius_value * cos(s);radius_value*sin(s); zeros(size(s))];
    dp = R * [-radius_value * sin(s);radius_value*cos(s); zeros(size(s))];
    ddp = R * [-cos(s)*radius_value; -sin(s)*radius_value; zeros(size(s))];
    dddp = R * [sin(s)*(radius_value);-cos(s)*(radius_value); zeros(size(s))];

%% frenet frames
T = R * [-radius_value .* sin(s);radius_value.*cos(s);zeros(size(s))];
T = T ./ norm(T);

N = R * [-radius_value .* cos(s);-radius_value .* sin(s);zeros(size(s));];
N = N ./ norm(N);

B = cross(T, N);
B = B ./ norm(B);

frenet_frames.T = T;
frenet_frames.N = N;
frenet_frames.B = B;

end