

// compute the alternately add and multiply sequence http://oeis.org/A039941
// 1,1,2,2,4,8,12,96,...
//
// it comes out backwards, but whatevs

function f(a,b) = a*b;
function g(a,b) = a+b;

function f_extend(L) = concat(f(L[0],L[1]), L);
function g_extend(L) = concat(g(L[0],L[1]), L);

function fgfg(L, times) = (times==0)?L:f_extend(gfgf(L,times-1));
function gfgf(L, times) = (times==0)?L:g_extend(fgfg(L,times-1));


echo( fgfg([1,1],10));
