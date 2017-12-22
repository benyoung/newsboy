// Bezier functions for openscad

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

// draw a little sphere, like a control point. turned off right now.
module node() {
    color("red")
    sphere(control_thickness, $fn=10);
}

// draw a "line" (really a cylinder) between 2 points in space.  turned off right now.
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

    color("pink")
    union(){
        for(i= [0:steps-1]) {
            line_segment(centers[i], centers[i+1], thickness);
            translate(centers[i])
            *sphere(thickness/2, $fn=16);
        }
        translate(centers[steps])
        *sphere(thickness/2, $fn=16);
    }
    
}



// 3d model of a "zigzag" triangulated surface spanning the space betweenh two bezier curves. turned off right now.
module two_bezier_patch(controlpoints_left, controlpoints_right, steps) {
    points_left  = [for(inc = [0:steps]) bezier3(controlpoints_left,  inc/steps)];
    points_right = [for(inc = [0:steps]) bezier3(controlpoints_right, inc/steps)];
    P = concat(points_left, points_right);
    
    triangles_1 = [for(i=[1:steps]) [steps+i, i, steps+i+1]];
    triangles_2 = [for(i=[0:steps-1]) [i+1,  i, steps+i+1]];
    T = concat(triangles_1, triangles_2);
    
    
    polyhedron(points=P, faces=T);
}

// angles A,B,C in triangle ABC
function angle_A(T) = ((T[1]==T[0])||(T[2]==T[0]))?0:acos((T[1]-T[0]) *(T[2]-T[0])/norm(T[1]-T[0])/norm(T[2]-T[0]));
function angle_B(T) = ((T[1]==T[0])||(T[2]==T[1]))?0:acos((T[0]-T[1]) *(T[2]-T[1])/norm(T[0]-T[1])/norm(T[2]-T[1]));
function angle_C(T) = ((T[0]==T[2])||(T[2]==T[1]))?0:acos((T[0]-T[2]) *(T[1]-T[2])/norm(T[0]-T[2])/norm(T[1]-T[2]));

// the standard angle, in plane, from point a to point b
function standard_angle(a,b) = (a==b)?0:atan2(b[1]-a[1], b[0]-a[0]);

// lengths of sides AB,BC,AC of triangle ABC
function len_BC(T) = norm(T[2]-T[1]);
function len_AC(T) = norm(T[2]-T[0]);
function len_AB(T) = norm(T[1]-T[0]);

// go to point p, face in direction theta and walk distance d
function go(p, theta, d)= p + [cos(theta), sin(theta)]*d;

// construct a triangle abc congruent to T=ABC in the plane, given pre-constructed points a, b
function construct_triangle_pos(T,a_p, b_p) = go(b_p,-angle_B(T)+standard_angle(b_p,a_p), len_BC(T));
function construct_triangle_neg(T,a_p, b_p) = go(b_p,+angle_B(T)+standard_angle(b_p,a_p), len_BC(T));

// construct a triangle by zigzagging from bezier Cl to Cr at times t1, t2
function zig(Cl,Cr,t1,t2) = [bezier3(Cl,t2), bezier3(Cr,t1), bezier3(Cl,t1)];
function zag(Cl,Cr,t1,t2) = [bezier3(Cr,t2), bezier3(Cl,t2), bezier3(Cr,t1)];

// flatten out one zig/zag step
function flat_zig(Cl, Cr, t, inc, a, b) = construct_triangle_pos(zig(Cl,Cr,t,t+inc),a,b);
function flat_zag(Cl, Cr, t, inc, a, b) = construct_triangle_neg(zag(Cl,Cr,t,t+inc),a,b);

// Prepend one zig/zag step onto a list
function flat_zig_extend(Cl, Cr, t, inc, L) = concat([flat_zig(Cl,Cr,t,inc,L[1],L[0])], L);
function flat_zag_extend(Cl, Cr, t, inc, L) = concat([flat_zag(Cl,Cr,t,inc,L[1],L[0])], L);

// Recursively prepend a bunch of zig/zag steps onto a list
function zigzag(Cl, Cr, n, steps, L) = (n==steps)?L:flat_zig_extend(Cl,Cr,n/steps,1/steps,zagzig(Cl,Cr,n,steps,L));
function zagzig(Cl, Cr, n, steps, L) = (n==steps)?L:flat_zag_extend(Cl,Cr,n/steps,1/steps,zigzag(Cl,Cr,n+1,steps,L));

function flatten_zigzag_path(S_l, S_r, steps) = zigzag(S_l, S_r,0,steps,[[0, norm(S_l[3]-S_r[3])], [0,0]] );

function unzigzag(L) = concat([for(i=[1:2:len(L)-1]) L[i]],[for(i=[len(L)-2:-2:0])L[i]]);

function flatten_patch(S_l, S_r, steps) = unzigzag(flatten_zigzag_path(S_l, S_r, steps));


