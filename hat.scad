include <bezier.scad>;

//low_spline = [[20,0,0],[10,0,0],[0,10,0],[0,20,0]];
//high_spline =[[30,0,10],[20,0,5],[0,20,5],[0,20,10]];
//polygon(points=flatten_patch(low_spline, high_spline, 20));
//two_bezier_patch(low_spline, high_spline, 20);
//echo(flatten_zigzag_path(low_spline, high_spline, 20));


mm=1;
cm = 10*mm;
in=25.4*mm;
eps = 0.01*mm;
function rotate_point_around_y_axis(pt, ang) = [pt[0] * cos(ang), pt[1], pt[2]*sin(ang)];
function rotate_around_line(pt, ang, x_coord) = rotate_point_around_y_axis(pt-[x_coord,0,0], ang)+[x_coord,0,0];


control_thickness = 2*mm;

rotate([0,8,0])
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
head_front_control_len = 2.4*in;
head_side_control_rear_len = 2.8*in;
head_side_control_front_len = 2.2*in;
head_side_control_angle = 3;


side_unit_vector = [cos(head_side_control_angle), -sin(head_side_control_angle), 0];


front_anchor=[head_front_len,0,0];
side_anchor=[0,head_side_len,0];
for(p = [rear_anchor, front_anchor,side_anchor]) {
    translate(p)
*    node();
}

rear_control = rear_anchor + [0,head_rear_control_len,0];

color("red")
line_segment(rear_control, rear_anchor, control_thickness);

front_control = front_anchor + [0,head_front_control_len,0];
color("red")
line_segment(front_control, front_anchor, control_thickness);

side_control_rear = side_anchor - side_unit_vector*head_side_control_rear_len;
side_control_front = side_anchor + side_unit_vector*head_side_control_front_len;
color("red")
line_segment(side_control_rear, side_control_front, control_thickness);

q1_brim = [front_anchor, front_control, side_control_front, side_anchor];
color("blue")
bezier_tube(q1_brim, 20, control_thickness);

q2_brim = [side_anchor, side_control_rear, rear_control, rear_anchor];
color("blue")
bezier_tube(q2_brim, 20, control_thickness);

brim_split_point = 0.8;
brim_length = 2.3*in;
brim_angle = 45;
brim_inv_angle = -45;
brim_control_width = 2.9*in;
brim_temple_pull_length = 1.3*in;

brim_tip_offset = brim_length * [cos(brim_angle),0,sin(brim_angle)];
brim_temple_pull = brim_temple_pull_length * [cos(brim_angle/2), 0, sin(brim_angle/2)];

brim_inv_tip_offset = brim_length * [cos(brim_inv_angle),0,sin(brim_inv_angle)];
brim_inv_temple_pull = brim_temple_pull_length * [cos(brim_inv_angle/2), 0, sin(brim_inv_angle/2)];


brim_forehead_spline = left_split(q1_brim, brim_split_point);
brim_side_spline = right_split(q1_brim, brim_split_point);

color("blue")
bezier_tube(brim_forehead_spline, 20, control_thickness);



brim_extend_anchor = brim_forehead_spline[0] + brim_tip_offset;
brim_inv_extend_anchor = brim_forehead_spline[0] + brim_inv_tip_offset;

brim_anchor = avg(brim_extend_anchor, front_anchor, 0.0);
brim_inv_anchor = avg(brim_inv_extend_anchor, front_anchor, 0.0);
brim_temple_anchor = brim_forehead_spline[3];
brim_temple_control = brim_temple_anchor + brim_inv_temple_pull;

brim_extend_control = brim_extend_anchor + [0,brim_control_width,0];
brim_inv_extend_control = brim_inv_extend_anchor + [0,brim_control_width,0];


brim_control = brim_anchor + [0,brim_control_width,0];
brim_inv_control = brim_inv_anchor + [0,brim_control_width,0];

color("red") {
    for(anchor = [brim_anchor, brim_temple_anchor])
    translate(anchor)
    node();
    
    line_segment(brim_temple_anchor, brim_temple_control, control_thickness);
    line_segment(brim_control, brim_anchor, control_thickness);
}

brim_edge_spline = [brim_anchor, brim_control, brim_temple_control, brim_temple_anchor];
brim_extend_spline = [brim_extend_anchor, brim_extend_control, brim_temple_control, brim_temple_anchor];

//bezier_tube(brim_edge_spline, 20, control_thickness);
//bezier_tube(brim_extend_spline, 20, control_thickness);
   
brim_inv_edge_spline = [brim_inv_anchor, brim_inv_control, brim_temple_control, brim_temple_anchor];
brim_inv_extend_spline = [brim_inv_extend_anchor, brim_inv_extend_control, brim_temple_control, brim_temple_anchor];

//bezier_tube(brim_inv_edge_spline, 20, control_thickness);
bezier_tube(brim_inv_extend_spline, 20, control_thickness);
 




top_anchor = [-0.7*in,0*in,4.3*in];
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

color("red") {
    translate(top_anchor)
    node();
    line_segment(top_control_rear, top_control_front, control_thickness);
}



color("red") {
    translate(upper_side_anchor)
    node();
    line_segment(upper_side_control_rear, upper_side_control_front, control_thickness);
}

peak_lift = [0*in, 0*in, 0*in];

peak_anchor = brim_edge_spline[0] + peak_lift;
peak_control_stretch = 1.3;
peak_control = peak_control_stretch * [0,brim_control_width,0] + peak_anchor;
peak_spline = [peak_anchor, peak_control, upper_side_control_front, upper_side_anchor];

peak_split_t = 0.6;
front_peak_spline = left_split(peak_spline, peak_split_t);
mid_peak_spline = right_split(peak_spline, peak_split_t);
color("red")
translate(mid_peak_spline[0])
node();


color("red")
line_segment(peak_anchor, peak_control, control_thickness);


bezier_tube(peak_spline, 20, control_thickness);

peak_crest_control_length = 2*in;
peak_crest_control_offset = peak_crest_control_length * [-sin(brim_angle), 0, cos(brim_angle)];
peak_crest_control = peak_anchor + peak_crest_control_offset;
front_crest_spline = [peak_anchor, peak_crest_control, top_control_front, top_anchor];
front_split_t = 0.5;
veryfront_crest_spline = left_split(front_crest_spline, front_split_t);
mid_crest_spline = right_split(front_crest_spline, front_split_t);

color("red")
line_segment(peak_anchor, peak_crest_control, control_thickness);


bezier_tube(front_crest_spline, 20, control_thickness);

rear_crest_control_len = 3*in;
rear_crest_control = rear_anchor + [0,0,rear_crest_control_len];
rear_crest_spline = [top_anchor, top_control_rear, rear_crest_control, rear_anchor];

bezier_tube(rear_crest_spline, 20, control_thickness);

rear_split_t = 0.7;
rear_edge_spline = right_split(q2_brim, rear_split_t);
rear_mid_spline = left_split(q2_brim, rear_split_t);

rear_split_point = rear_edge_spline[0];
color("red")
translate(rear_split_point)
node();

rear_split_control_len = 2*in;
rear_split_control = rear_split_point + [0,0,rear_split_control_len];
rear_upperpanel_spline = [upper_side_anchor, upper_side_control_rear, rear_split_control, rear_split_point];


bezier_tube(rear_upperpanel_spline, 20, control_thickness);

module half_hat() {
    color("blue") {
 //       two_bezier_patch(brim_edge_spline, front_peak_spline, 20);
        two_bezier_patch(brim_forehead_spline, front_peak_spline, 20);
        two_bezier_patch(brim_side_spline, mid_peak_spline, 20);
        two_bezier_patch(rear_mid_spline, rear_upperpanel_spline, 20);
        
        two_bezier_patch(front_peak_spline, veryfront_crest_spline, 20);
        two_bezier_patch(mid_peak_spline, mid_crest_spline, 20);
        two_bezier_patch(rear_upperpanel_spline, rear_crest_spline, 20);
        
 //       two_bezier_patch(brim_edge_spline, brim_forehead_spline,20);
        two_bezier_patch(brim_inv_extend_spline, brim_forehead_spline,20);
    }
}

translate([0,0,0*in]) {
half_hat();
mirror([0,1,0])
half_hat();
}
echo("hatband size is", 2 * (arclength(q1_brim, 40) + arclength(q2_brim, 40)), "millimeters");


module flat_half_brim() {
    pts = flatten_patch(brim_inv_extend_spline, brim_forehead_spline,20);
    n = len(pts);
    last = pts[0]-pts[n-1];

    rotate(-atan2(last[1], last[0]))
    translate(-pts[n-1])
    polygon(pts);
}

/*module flat_half_hidden_bit() {
    pts = flatten_patch(brim_edge_spline, brim_forehead_spline,20);
    n = len(pts);
    last = pts[0]-pts[n-1];

    rotate(-atan2(last[1], last[0]))
    translate(-pts[n-1])
    polygon(pts);
    
}
module flat_hidden_bit() {
    union(){
        flat_half_hidden_bit();
        translate([0,-eps])
        mirror([0,1])
        flat_half_hidden_bit();
    }
}
*/

module flat_brim() {
    union(){
        flat_half_brim();
        translate([0,-eps])
        mirror([0,1])
        flat_half_brim();
    }
}


module flat_top_half() {
    pts_f = flatten_patch(front_peak_spline, veryfront_crest_spline, 20);
    pts_m = flatten_patch(mid_peak_spline, mid_crest_spline, 20);
    pts_r = flatten_patch(rear_upperpanel_spline, rear_crest_spline, 20);   
    n = len(pts_f);
    
    last_m = pts_m[0]-pts_m[n-1];
    last_r = pts_r[0]-pts_r[n-1];
    //rotate(-atan2(last_f[1], last_f[0]))
    //translate(-pts_f[n-1])
    union(){
        translate(pts_m[0])
        rotate(atan2(last_m[1], last_m[0])+90)
        union(){
            polygon(pts_f);
            translate([eps,0])
            rotate(-atan2(last_m[1], last_m[0])-90)
            translate(-pts_m[0])
            polygon(pts_m);
        }
        translate([eps,0])
        rotate(-atan2(last_r[1], last_r[0])-90)
        translate(-pts_r[0])
        polygon(pts_r);
    }
}

top_panel_fix_rot = 9.45;
module flat_top_panel() {
    union(){
        rotate(-top_panel_fix_rot)
        flat_top_half();
        mirror([0,1])
        rotate(-top_panel_fix_rot)
        flat_top_half();
    }
}


module flat_side() {
    pts_f = flatten_patch(brim_forehead_spline, front_peak_spline, 20);
    pts_m = flatten_patch(brim_side_spline, mid_peak_spline, 20);
    pts_r = flatten_patch(rear_mid_spline, rear_upperpanel_spline, 20);   
  
    n = len(pts_f);
    
    last_m = pts_m[0]-pts_m[n-1];
    last_r = pts_r[0]-pts_r[n-1];

    union(){
        translate(pts_m[0])
        rotate(atan2(last_m[1], last_m[0])+90)
        union(){
            polygon(pts_f);
            translate([eps,0])
            rotate(-atan2(last_m[1], last_m[0])-90)
            translate(-pts_m[0])
            polygon(pts_m);
        }
        translate([eps,0])
        rotate(-atan2(last_r[1], last_r[0])-90)
        translate(-pts_r[0])
        polygon(pts_r);

    }
    
}



seam_allowance = 0.5*in;
translate([4*in,4.5*in])
rotate(83)
difference() {
    offset(r=seam_allowance)
    flat_side();
    flat_side();
}

translate([8*in,4.5*in])
mirror([0,1])
rotate(83+180)
difference() {
    offset(r=seam_allowance)
    flat_side();
    flat_side();
}



translate([4.0*in,16.5*in])
rotate(-90)
union(){
   
    *square(4*in);
    translate([-1*in,2*in])
    difference() {
        offset(r=seam_allowance)
        flat_top_panel();
        flat_top_panel();
    }
}

for(trans=[-1*in,-5*in])

translate([4*in,trans])
rotate(-90)
difference(){
    offset(r=seam_allowance)
    flat_brim();
    flat_brim();
}
/*
translate([0,6.5*in])
rotate(-90)
difference(){
    offset(r=seam_allowance)
    flat_hidden_bit();
    flat_hidden_bit();
}

*/
*translate([0,0,-0.15*in])
color("green")
cube([12*in, 24*in,0.1*in]);
