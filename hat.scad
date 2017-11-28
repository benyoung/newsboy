// cubic bezier basis functions
function b0(t) = (1-t)*(1-t)*(1-t);
function b1(t) = 3*(1-t)*(1-t)*t;
function b2(t) = 3*(1-t)*t*t;
function b3(t) = t*t*t;

function bezier(pts, t) = pts[0]*b0(t) + pts[1]*b1(t) + pts[2]*b2(t) + pts[3]*b3(t);
function bezier3(pts, t) = [for(coord = [0:2]) bezier([for(i=[0:3]) pts[i][coord]], t)];
    


module line_segment(p1, p2, thickness) {
    v = p2-p1;
    rho = norm(v);
    theta = acos(v[2]/rho);
    phi = atan2(v[1], v[0]);
    
    translate(p1)
    rotate([0,0,phi])
    rotate([0,theta,0])
    cylinder(h=rho, r=thickness/2, $fn=16);
}

module bezier_curve(control_points, steps, thickness) {
    centers = [for(inc = [0:steps]) bezier3(control_points, inc/steps)];
    color("red")
    union(){
        for(i= [0:steps-1]) {
            color("red")
            line_segment(centers[i], centers[i+1], thickness);
            translate(centers[i])
            sphere(thickness/2, $fn=16);
        }
        translate(centers[steps])
        sphere(thickness/2, $fn=16);
    }
    
}

// concatenate lists - from openscad wiki
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];

module two_bezier_patch(controlpoints_left, controlpoints_right, steps) {
    points_left  = [for(inc = [0:steps]) bezier3(controlpoints_left,  inc/steps)];
    points_right = [for(inc = [0:steps]) bezier3(controlpoints_right, inc/steps)];
    P = cat(points_left, points_right);
    
    triangles_1 = [for(i=[0:steps-1]) [steps+i, i, steps+i+1]];
    triangles_2 = [for(i=[0:steps-1]) [i+1,  i, steps+i+1]];
    T = cat(triangles_1, triangles_2);
    
    
    polyhedron(points=P, faces=T);
}

steps=10;

control_points_left = [[0,0,0],[10,0,0],[10,0,10],[10,10,10]];
bezier_curve(control_points_left, steps);
control_points_right = [[0,0,0],[0,10,0],[0,10,10],[10,10,10]];
bezier_curve(control_points_right, steps);

two_bezier_patch(control_points_left, control_points_right, steps);