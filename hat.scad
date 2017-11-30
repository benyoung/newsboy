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

function sum(list, idx = 0, result = 0) = idx >= len(list) ? result : sum(list, idx + 1, result + list[idx]);  
function arclength(pts, n) = sum([for(i=[0:n-1]) norm(bezier(pts, (i+1)/n) - bezier(pts, i/n))]);

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



// 3d model of a "zigzag" triangulated surface spanning the space betweenh two bezier curves.
module two_bezier_patch(controlpoints_left, controlpoints_right, steps) {
    points_left  = [for(inc = [0:steps]) bezier3(controlpoints_left,  inc/steps)];
    points_right = [for(inc = [0:steps]) bezier3(controlpoints_right, inc/steps)];
    P = concat(points_left, points_right);
    
    triangles_1 = [for(i=[1:steps]) [steps+i, i, steps+i+1]];
    triangles_2 = [for(i=[0:steps-1]) [i+1,  i, steps+i+1]];
    T = concat(triangles_1, triangles_2);
    
    
    polyhedron(points=P, faces=T);
}




mm=1;
cm = 10*mm;
in=25.4*mm;

control_thickness = 2*mm;

*rotate([0,8,0])
translate([0,0,-7.5*in])
scale(25.4)
rotate([0,0,90])
import("HeadStand.STL");  
// I think this model was based on inches as the dimensionless unit
// at least it is consistent with an average male interpupillary distance of 64mm

//splines for hatband. these are distances from origin to various points on the head
head_rear_len = 4.3*in;
head_front_len = 4.1*in;
head_side_len = 3.3*in;
rear_anchor = [-head_rear_len, 0,0];


head_rear_control_len = 1.7*in;
head_front_control_len = 2.1*in;
head_side_control_rear_len = 2.8*in;
head_side_control_front_len = 2.5*in;
head_side_control_angle = 3;


side_unit_vector = [cos(head_side_control_angle), -sin(head_side_control_angle), 0];


front_anchor=[head_front_len,0,0];
side_anchor=[0,head_side_len,0];
*for(p = [rear_anchor, front_anchor,side_anchor]) {
    translate(p)
    color("red")
    sphere(control_thickness, $fn=16);
}

rear_control = rear_anchor + [0,head_rear_control_len,0];

*color("red")
line_segment(rear_control, rear_anchor, control_thickness);

front_control = front_anchor + [0,head_front_control_len,0];
*color("red")
line_segment(front_control, front_anchor, control_thickness);

side_control_rear = side_anchor - side_unit_vector*head_side_control_rear_len;
side_control_front = side_anchor + side_unit_vector*head_side_control_front_len;
*color("red")
line_segment(side_control_rear, side_control_front, control_thickness);

q1_brim = [front_anchor, front_control, side_control_front, side_anchor];
*color("blue")
bezier_tube(q1_brim, 20, control_thickness);

q2_brim = [side_anchor, side_control_rear, rear_control, rear_anchor];
*color("blue")
bezier_tube(q2_brim, 20, control_thickness);

brim_split_point = 0.8;
brim_tip_offset = [1.3*in,0*in,-1*in];
brim_control_width = 2.2*in;
brim_temple_pull = [1*in,0*in,-0.5*in];

brim_forehead_spline = left_split(q1_brim, brim_split_point);
brim_side_spline = right_split(q1_brim, brim_split_point);
*color("blue")
bezier_tube(brim_forehead_spline, 20, control_thickness);

brim_anchor = brim_forehead_spline[0] + brim_tip_offset;
brim_temple_anchor = brim_forehead_spline[3];
brim_temple_control = brim_temple_anchor + brim_temple_pull;

brim_control = brim_anchor + [0,brim_control_width,0];
*color("red") {
    for(anchor = [brim_anchor, brim_temple_anchor])
    translate(anchor)
    sphere(control_thickness, $fn=16);
    
    line_segment(brim_temple_anchor, brim_temple_control, control_thickness);
    line_segment(brim_control, brim_anchor, control_thickness);
}

brim_edge_spline = [brim_anchor, brim_control, brim_temple_control, brim_temple_anchor];
*color("blue")
bezier_tube(brim_edge_spline, 20, control_thickness);
    




top_anchor = [-0.7*in,0*in,4.0*in];
top_control_front_len = 3*in;
top_control_front = top_anchor + top_control_front_len*[1,0,0];
top_control_rear_len = 2.5*in;
top_control_rear = top_anchor + top_control_rear_len*[-1,0,0];

upper_side_anchor = top_anchor + [0.3*in, 2.9*in, -0.5*in];
upper_side_rear_control_len = 2*in; 
upper_side_front_control_len = 3.8*in;
upper_side_angle = 8;
upper_side_unit_vector = [cos(upper_side_angle), sin(upper_side_angle), 0];
upper_side_control_front = upper_side_anchor + upper_side_front_control_len*upper_side_unit_vector;
upper_side_control_rear = upper_side_anchor - upper_side_rear_control_len*upper_side_unit_vector;

*color("red") {
    translate(top_anchor)
    sphere(control_thickness);
    line_segment(top_control_rear, top_control_front, control_thickness);
}



*color("red") {
    translate(upper_side_anchor)
    sphere(control_thickness);
    line_segment(upper_side_control_rear, upper_side_control_front, control_thickness);
}

peak_lift = [0.1*in, 0, 0.6*in];

peak_anchor = brim_edge_spline[0] + peak_lift;
peak_control_stretch = 1.6;
peak_control = peak_control_stretch * [0,brim_control_width,0] + peak_anchor;
peak_spline = [peak_anchor, peak_control, upper_side_control_front, upper_side_anchor];

peak_split_t = 0.6;
front_peak_spline = left_split(peak_spline, peak_split_t);
mid_peak_spline = right_split(peak_spline, peak_split_t);
*color("red")
translate(mid_peak_spline[0])
sphere(control_thickness);


*color("red")
line_segment(peak_anchor, peak_control, control_thickness);

*color("blue")
bezier_tube(peak_spline, 20, control_thickness);

peak_crest_control_offset = [-1.1*in, 0, 1.7*in];
peak_crest_control = peak_anchor + peak_crest_control_offset;
front_crest_spline = [peak_anchor, peak_crest_control, top_control_front, top_anchor];
front_split_t = 0.5;
veryfront_crest_spline = left_split(front_crest_spline, front_split_t);
mid_crest_spline = right_split(front_crest_spline, front_split_t);

*color("red")
line_segment(peak_anchor, peak_crest_control, control_thickness);

*color("blue")
bezier_tube(front_crest_spline, 20, control_thickness);

rear_crest_control_len = 3*in;
rear_crest_control = rear_anchor + [0,0,rear_crest_control_len];
rear_crest_spline = [top_anchor, top_control_rear, rear_crest_control, rear_anchor];
*color("blue")
bezier_tube(rear_crest_spline, 20, control_thickness);

rear_split_t = 0.7;
rear_edge_spline = right_split(q2_brim, rear_split_t);
rear_mid_spline = left_split(q2_brim, rear_split_t);

rear_split_point = rear_edge_spline[0];
*color("red")
translate(rear_split_point)
sphere(r=control_thickness);

rear_split_control_len = 2*in;
rear_split_control = rear_split_point + [0,0,rear_split_control_len];
rear_upperpanel_spline = [upper_side_anchor, upper_side_control_rear, rear_split_control, rear_split_point];

*color("blue")
bezier_tube(rear_upperpanel_spline, 20, control_thickness);

module half_hat() {
    color("blue") {
        two_bezier_patch(brim_edge_spline, front_peak_spline, 20);
        two_bezier_patch(brim_side_spline, mid_peak_spline, 20);
        two_bezier_patch(rear_mid_spline, rear_upperpanel_spline, 20);
        
        two_bezier_patch(front_peak_spline, veryfront_crest_spline, 20);
        two_bezier_patch(mid_peak_spline, mid_crest_spline, 20);
        two_bezier_patch(rear_upperpanel_spline, rear_crest_spline, 20);
        
        two_bezier_patch(brim_edge_spline, brim_forehead_spline,20);
    }
}

half_hat();
mirror([0,1,0])
half_hat();

echo("hatband size is", 2 * (arclength(q1_brim, 40) + arclength(q2_brim, 40)), "millimeters");




