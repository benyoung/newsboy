// cubic bezier basis functions
function b0(t) = (1-t)*(1-t)*(1-t);
function b1(t) = 3*(1-t)*(1-t)*t;
function b2(t) = 3*(1-t)*t*t;
function b3(t) = t*t*t;

// find the point at position t (0 < t < 1) on a cubic bezier curve defined by 4 control points
function bezier(pts, t) = pts[0]*b0(t) + pts[1]*b1(t) + pts[2]*b2(t) + pts[3]*b3(t);
function bezier3(pts, t) = [for(coord = [0:2]) bezier([for(i=[0:3]) pts[i][coord]], t)];
    
// de casteljau algorithm for splitting bezier curves via repeated weighted averaging. 
// horribly inefficient b/c no dynamic programming. Oh well. Use sparingly.
function avg(p0, p1, t) = p0 * (1-t) + p1 * t;

function left_split(cp, t) = [
    cp[0],
    avg(cp[0], cp[1], t),
    avg(avg(cp[0], cp[1], t), avg(cp[1], cp[2], t),t),
    avg(avg(avg(cp[0], cp[1], t), avg(cp[1], cp[2], t),t), avg(avg(cp[1], cp[2], t), avg(cp[2], cp[3], t),t),t)
];

function right_split(cp, t) = [
    avg(avg(avg(cp[3], cp[2], 1-t), avg(cp[2], cp[1], 1-t),1-t), avg(avg(cp[2], cp[1], 1-t), avg(cp[1], cp[0], 1-t),1-t),1-t),
    avg(avg(cp[3], cp[2], 1-t), avg(cp[2], cp[1], 1-t),1-t),
    avg(cp[3], cp[2], 1-t),
    cp[3]
];

// draw a "line" (really a cylinder) between 2 points in space.
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

// 3d model of a tubular neighbourhood of a bezier curve.
module bezier_tube(control_points, steps, thickness) {
    centers = [for(inc = [0:steps]) bezier3(control_points, inc/steps)];

    union(){
        for(i= [0:steps-1]) {
            line_segment(centers[i], centers[i+1], thickness);
            translate(centers[i])
            sphere(thickness/2, $fn=16);
        }
        translate(centers[steps])
        sphere(thickness/2, $fn=16);
    }
    
}

// function to concatenate lists - from openscad wiki
function cat(L1, L2) = [for(L=[L1, L2], a=L) a];


// 3d model of a "zigzag" triangulated surface spanning the space betweenh two bezier curves.
module two_bezier_patch(controlpoints_left, controlpoints_right, steps) {
    points_left  = [for(inc = [0:steps]) bezier3(controlpoints_left,  inc/steps)];
    points_right = [for(inc = [0:steps]) bezier3(controlpoints_right, inc/steps)];
    P = cat(points_left, points_right);
    
    triangles_1 = [for(i=[0:steps-1]) [steps+i, i, steps+i+1]];
    triangles_2 = [for(i=[0:steps-1]) [i+1,  i, steps+i+1]];
    T = cat(triangles_1, triangles_2);
    
    
    polyhedron(points=P, faces=T);
}

//steps=10;
//
//control_points_left = [[0,0,0],[10,0,0],[10,0,10],[10,10,10]];
//split_time = 0.7;
//cp1 = left_split(control_points_left, split_time);
//color("red")
//bezier_tube(cp1, steps);
//cp2 = right_split(control_points_left, split_time);
//color("green")
//bezier_tube(cp2, steps);
//control_points_right = [[0,0,0],[0,10,0],[0,10,10],[10,10,10]];
//bezier_tube(control_points_right, steps);
//two_bezier_patch(control_points_left, control_points_right, steps);


mm=1;
cm = 10*mm;
in=25.4*mm;

translate([0,0,-7.5*in])
scale(25.4)
rotate([0,0,180])
import("HeadStand.STL");  
// I think this model was based on inches as the dimensionless unit
// at least it is consistent with an average male interpupillary distance of 64mm









