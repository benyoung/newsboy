include <bezier.scad>;


mm=1;
cm = 10*mm;
in=25.4*mm;
eps = 0.01*mm;
function rotate_point_around_y_axis(pt, ang) = [pt[0] * cos(ang) - pt[2]*sin(ang), pt[1], pt[0]*sin(ang) + pt[2]*cos(ang)];
function rotate_around_line(pt, ang, x_coord) = rotate_point_around_y_axis(pt-[x_coord,0,0], ang)+[x_coord,0,0];

//echo(rotate_around_line([3,0,2], 45, 1));

control_thickness = 3*mm;

// I think this model was based on inches as the dimensionless unit
// at least it is consistent with an average male interpupillary distance of 64mm
*rotate([0,8,0])
translate([0,0,-7*in])
scale(25.4)
rotate([0,0,90])
import("HeadStand.STL");  



// hatband dimensions. these are distances from origin to various points on the head
head_rear_len = 4.3*in;
head_front_len = 4.1*in;
head_side_len = 3.4*in;
rear_anchor = [-head_rear_len, 0,0];

head_rear_control_len = 2.2*in;
head_front_control_len = 2.4*in;
head_side_control_rear_len = 2.8*in;
head_side_control_front_len = 2.2*in;
head_side_control_angle = -1;

side_unit_vector = [cos(head_side_control_angle), -sin(head_side_control_angle), 0];


front_anchor=[head_front_len,0,0];
side_anchor=[0,head_side_len,0];




rear_control = rear_anchor + [0,head_rear_control_len,0];
front_control = front_anchor + [0,head_front_control_len,0];
side_control_rear = side_anchor - side_unit_vector*head_side_control_rear_len;
side_control_front = side_anchor + side_unit_vector*head_side_control_front_len;
q1_brim = [front_anchor, front_control, side_control_front, side_anchor];
q2_brim = [side_anchor, side_control_rear, rear_control, rear_anchor];
brim_split_point = 0.8;
brim_length = 2.7*in;
brim_angle = -35;
brim_inv_angle = -35;
brim_control_width = 4.1*in;
brim_temple_pull_length = 1.7*in;
brim_tip_offset = brim_length * [cos(brim_angle),0,sin(brim_angle)];
brim_temple_pull = brim_temple_pull_length * [cos(brim_angle/2), 0, sin(brim_angle/2)];
brim_inv_tip_offset = brim_length * [cos(brim_inv_angle),0,sin(brim_inv_angle)];
brim_inv_temple_pull = brim_temple_pull_length * [cos(brim_inv_angle/2), 0, sin(brim_inv_angle/2)];
brim_forehead_spline = left_split(q1_brim, brim_split_point);
brim_side_spline = right_split(q1_brim, brim_split_point);






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

brim_edge_spline = [brim_anchor, brim_control, brim_temple_control, brim_temple_anchor];
brim_extend_spline = [brim_extend_anchor, brim_extend_control, brim_temple_control, brim_temple_anchor];

   
brim_inv_edge_spline = [brim_inv_anchor, brim_inv_control, brim_temple_control, brim_temple_anchor];
brim_inv_extend_spline = [brim_inv_extend_anchor, brim_inv_extend_control, brim_temple_control, brim_temple_anchor];

 

rear_inversion_angle = angle_B([brim_extend_anchor, rear_anchor, brim_inv_extend_anchor]);
//rear_inversion_angle=0;

echo("rear inversion angle", rear_inversion_angle);

top_anchor_raw = [-0.3*in,0*in,4.4*in];
top_control_front_len = 3*in;
top_control_front_raw = top_anchor_raw + top_control_front_len*[1,0,0];
top_control_rear_len = 2.5*in;
top_control_rear_raw = top_anchor_raw + top_control_rear_len*[-1,0,0];

// rotate it
top_anchor = rotate_around_line(top_anchor_raw, rear_inversion_angle, rear_anchor[0]);
top_control_rear = rotate_around_line(top_control_rear_raw, rear_inversion_angle, rear_anchor[0]);
top_control_front = rotate_around_line(top_control_front_raw, rear_inversion_angle, rear_anchor[0]);



upper_side_anchor_raw = top_anchor + [-0.3*in, 2.5*in, -0.7*in];
upper_side_rear_control_len = 2.5*in; 
upper_side_front_control_len = 4.8*in;
upper_side_angle = 8;
upper_side_unit_vector = [cos(upper_side_angle), sin(upper_side_angle), 0];
upper_side_control_front_raw = upper_side_anchor_raw + upper_side_front_control_len*upper_side_unit_vector;
upper_side_control_rear_raw = upper_side_anchor_raw - upper_side_rear_control_len*upper_side_unit_vector;

// rotate it
upper_side_anchor = rotate_around_line(upper_side_anchor_raw, rear_inversion_angle, rear_anchor[0]);
upper_side_control_front = rotate_around_line(upper_side_control_front_raw, rear_inversion_angle, rear_anchor[0]);
upper_side_control_rear = rotate_around_line(upper_side_control_rear_raw, rear_inversion_angle, rear_anchor[0]);



peak_lift = [0*in, 0*in, 0*in];

peak_anchor = brim_edge_spline[0] + peak_lift;
peak_control_stretch = 1.0;
peak_control = peak_control_stretch * [0,brim_control_width,0] + peak_anchor;
peak_spline = [peak_anchor, peak_control, upper_side_control_front, upper_side_anchor];

peak_split_t = 0.6;
front_peak_spline = left_split(peak_spline, peak_split_t);
mid_peak_spline = right_split(peak_spline, peak_split_t);

peak_flatten_angle = -40;
peak_crest_control_length = 2*in;
peak_crest_control_offset = peak_crest_control_length * [sin(peak_flatten_angle-brim_angle), 0, cos(peak_flatten_angle-brim_angle)];
peak_crest_control = peak_anchor + peak_crest_control_offset;
front_crest_spline = [peak_anchor, peak_crest_control, top_control_front, top_anchor];
front_split_t = 0.5;
veryfront_crest_spline = left_split(front_crest_spline, front_split_t);
mid_crest_spline = right_split(front_crest_spline, front_split_t);




rear_crest_control_len = 3*in;
rear_crest_control_raw = rear_anchor + [0,0,rear_crest_control_len];
rear_crest_control=rotate_around_line(rear_crest_control_raw, rear_inversion_angle, rear_anchor[0]);

echo(rear_crest_control_raw, rear_crest_control);
rear_crest_spline = [top_anchor, top_control_rear, rear_crest_control, rear_anchor];


rear_split_t = 0.65;
rear_edge_spline = right_split(q2_brim, rear_split_t);
rear_mid_spline = left_split(q2_brim, rear_split_t);

rear_split_point = rear_edge_spline[0];


rear_split_control_len = 2*in;
rear_split_control_raw = rear_split_point + [0,0,rear_split_control_len];

rear_split_control = rotate_around_line(rear_split_control_raw, rear_inversion_angle, rear_anchor[0]);
rear_upperpanel_spline = [upper_side_anchor, upper_side_control_rear, rear_split_control, rear_split_point];

// Flattened panels

module flat_half_brim() {
    pts = flatten_patch(brim_inv_extend_spline, brim_forehead_spline,20);
    n = len(pts);
    last = pts[0]-pts[n-1];

    rotate(-atan2(last[1], last[0]))
    translate(-pts[n-1])
    polygon(pts);
}

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

top_panel_fix_rot = 2.8;
top_panel_sep = 6*mm;
nose_fix_rad = 80*mm;
nose_fix_offset = [165*mm,10*mm];
module flat_top_panel(){
    offset(0.1*mm)
    union(){
        translate([0,top_panel_sep])
        rotate(-top_panel_fix_rot)
        flat_top_half();
        translate([0,-top_panel_sep])
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


seam_allowance = 0.25*in;
stich_line_offset = 0;// 0.54*in / 2; i forget why i did this but it sucks




// pattern pieces with layout
union(){
    
    translate([11*in,4.2*in])
    rotate(100)
    mirror([0,1])
    difference() {
        offset(r=seam_allowance)
        flat_side();
        offset(r=stich_line_offset)
        flat_side();
    }
    

       
    translate([1.7*in,13*in])
    rotate(-90)
    union(){
       
        *square(4*in);
        translate([-1*in,2*in])
        difference() {
            offset(r=seam_allowance)
            flat_top_panel();
            offset(r=stich_line_offset)
            flat_top_panel();
        }
    }
    
    

    translate([10.0*in,21*in])
    rotate(90)
    difference(){
        offset(r=seam_allowance)
        flat_brim();
        offset(r=stich_line_offset)
        flat_brim();
    }
    


}


/* 2 foot by 1 foot cut bed */
*
translate([0,0,-0.15*in])
color("green")
cube([12*in, 24*in,0.1*in]);
*
color("cyan")
translate([0,-24*in,-0.15*in])
cube([12*in, 24*in,0.1*in]);

/* measurements to see how much fabric we actually need */
/*
measure_1 = 16*in;
measure_2 = 16*in;
translate([0,24*in-measure_1,-0.10*in])
color("red")
cube([12*in, measure_1,0.1*in]);
translate([0,-measure_2,-0.10*in])
color("blue")
cube([12*in, measure_2,0.1*in]);
*/

/* bezier patches for preview */

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

*
translate([0,0,0*in]) {
half_hat();
mirror([0,1,0])
half_hat();
}
echo("hatband size is", 2 * (arclength(q1_brim, 40) + arclength(q2_brim, 40)), "millimeters");

// anchor points and handles
*
color("red") {
    line_segment(rear_control, rear_anchor, control_thickness);
    line_segment(front_control, front_anchor, control_thickness);
    line_segment(side_control_rear, side_control_front, control_thickness);
    line_segment(brim_temple_anchor, brim_temple_control, control_thickness);
    line_segment(brim_control, brim_anchor, control_thickness);
    line_segment(peak_anchor, peak_control, control_thickness);
    line_segment(peak_anchor, peak_crest_control, control_thickness);
    line_segment(upper_side_control_rear, upper_side_control_front, control_thickness); 
    line_segment(top_control_rear, top_control_front, control_thickness); 
    
    translate(top_anchor) node(); 
    translate(mid_peak_spline[0]) node();
    translate(rear_split_point) node();
    translate(upper_side_anchor) node();
    translate(brim_anchor) node();
    translate(brim_temple_anchor) node();
    translate(rear_anchor) node();
    translate(front_anchor) node();
    translate(side_anchor) node();
}

// bezier tubes / wireframe for hat
*
color("pink") {
    bezier_tube(q1_brim, 20, control_thickness);
    bezier_tube(q2_brim, 20, control_thickness);
    bezier_tube(brim_forehead_spline, 20, control_thickness);
    bezier_tube(brim_edge_spline, 20, control_thickness);
    bezier_tube(brim_extend_spline, 20, control_thickness);
    bezier_tube(brim_inv_edge_spline, 20, control_thickness);
    bezier_tube(brim_inv_extend_spline, 20, control_thickness);
    bezier_tube(peak_spline, 20, control_thickness);
    bezier_tube(front_crest_spline, 20, control_thickness);
    bezier_tube(rear_crest_spline, 20, control_thickness);
    bezier_tube(rear_upperpanel_spline, 20, control_thickness);
}
