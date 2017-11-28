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
    
    triangles_1 = [for(i=[1:steps-1]) [steps+i, i, steps+i+1]];
    triangles_2 = [for(i=[0:steps-1]) [i+1,  i, steps+i+1]];
    T = cat(triangles_1, triangles_2);
    
    
    polyhedron(points=P, faces=T);
}



mm=1;
cm = 10*mm;
in=25.4*mm;

rotate([0,8,0])
translate([0,0,-7.5*in])
scale(25.4)
rotate([0,0,90])
import("HeadStand.STL");  
// I think this model was based on inches as the dimensionless unit
// at least it is consistent with an average male interpupillary distance of 64mm

//splines for hatband. these are distances from origin to various points on the head
head_rear_len = 4.15*in;
head_front_len = 4.0*in;
head_side_len = 3.05*in;

head_rear_control_len = 1.7*in;
head_front_control_len = 2.1*in;
head_side_control_back_len = 2.8*in;
head_side_control_front_len = 2.5*in;
head_side_control_angle = 3;

head_left_unit_vector = [cos(head_side_control_angle), -sin(head_side_control_angle), 0];
head_right_unit_vector = [cos(head_side_control_angle), sin(head_side_control_angle), 0];
rear_anchor =[-head_rear_len,0,0];
front_anchor=[head_front_len,0,0];
left_anchor=[0,head_side_len,0];
right_anchor=[0,-head_side_len,0];
*for(p = [rear_anchor, front_anchor,left_anchor,right_anchor]) {
    translate(p)
    color("red")
    sphere(5*mm, $fn=16);
}

rear_control_left = rear_anchor + [0,head_rear_control_len,0];
rear_control_right = rear_anchor + [0,-head_rear_control_len,0];
*color("red")
line_segment(rear_control_left, rear_control_right, 5*mm);

front_control_left = front_anchor + [0,head_front_control_len,0];
front_control_right = front_anchor + [0,-head_front_control_len,0];
*color("red")
line_segment(front_control_left, front_control_right, 5*mm);

left_control_back = left_anchor - head_left_unit_vector*head_side_control_back_len;
left_control_front = left_anchor + head_left_unit_vector*head_side_control_front_len;
*color("red")
line_segment(left_control_back, left_control_front, 5*mm);

right_control_back = right_anchor - head_right_unit_vector*head_side_control_back_len;
right_control_front = right_anchor + head_right_unit_vector*head_side_control_front_len;
*color("red")
line_segment(right_control_back, right_control_front, 5*mm);

q1_brim = [front_anchor, front_control_left, left_control_front, left_anchor];
color("blue")
bezier_tube(q1_brim, 20, 5*mm);

q2_brim = [left_anchor, left_control_back, rear_control_left, rear_anchor];
color("blue")
bezier_tube(q2_brim, 20, 5*mm);

q3_brim = [right_anchor, right_control_back, rear_control_right, rear_anchor];
color("blue")
bezier_tube(q3_brim, 20, 5*mm);

q4_brim = [front_anchor, front_control_right, right_control_front, right_anchor];
color("blue")
bezier_tube(q4_brim, 20, 5*mm);

brim_split_point = 0.58;
brim_tip_offset = [1.3*in,0*in,-1*in];
brim_control_width = 2.2*in;
brim_left_temple_pull = [1*in,0*in,-0.5*in];
brim_right_temple_pull = brim_left_temple_pull - [0,2*brim_left_temple_pull[1],0];

left_brim_forehead_spline = left_split(q1_brim, brim_split_point);
*color("blue")
bezier_tube(left_brim_forehead_spline, 20, 5*mm);

right_brim_forehead_spline = left_split(q4_brim, brim_split_point);
*color("blue")
bezier_tube(right_brim_forehead_spline, 20, 5*mm);

brim_anchor = left_brim_forehead_spline[0] + brim_tip_offset;
brim_left_temple_anchor = left_brim_forehead_spline[3];
brim_left_temple_control = brim_left_temple_anchor + brim_left_temple_pull;

brim_right_temple_anchor = right_brim_forehead_spline[3];
brim_right_temple_control = brim_right_temple_anchor + brim_right_temple_pull;

brim_left_control = brim_anchor + [0,brim_control_width,0];
brim_right_control = brim_anchor - [0,brim_control_width,0];
*color("red") {
    for(anchor = [brim_anchor, brim_left_temple_anchor, brim_right_temple_anchor])
    translate(anchor)
    sphere(5*mm, $fn=16);
    
    line_segment(brim_left_temple_anchor, brim_left_temple_control, 5*mm);
    line_segment(brim_right_temple_anchor, brim_right_temple_control, 5*mm);
    line_segment(brim_left_control, brim_right_control, 5*mm);
}

left_brim_edge_spline = [brim_anchor, brim_left_control, brim_left_temple_control, brim_left_temple_anchor];
color("blue")
bezier_tube(left_brim_edge_spline, 20, 5*mm);

right_brim_edge_spline = [brim_anchor, brim_right_control, brim_right_temple_control, brim_right_temple_anchor];
color("blue")
bezier_tube(right_brim_edge_spline, 20, 5*mm);
    
color("blue")
two_bezier_patch(left_brim_edge_spline, left_brim_forehead_spline,20);
color("blue")
two_bezier_patch(right_brim_edge_spline, right_brim_forehead_spline,20);



