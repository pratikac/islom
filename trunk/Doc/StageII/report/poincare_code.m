poincareMap[{t0_, x0_, y0_, len0_, theta0_, xdot0_, ydot0_, 
lendot0_, thetadot0_}] := Module[{pInit},

pInit = {t0, x0, y0, len0, theta0, xdot0, ydot0, lendot0, thetadot0};
{op1, pt1, pt2} = stancePhase[pInit];
{op2, pt1, pt2} = flightPhase[op1];
{op3, pt1, pt2} = springPhase[op2];
op3[[1]] = 0;
op3[[2]] = 0;

op3
];

cost[op1_List, op2_List] := Module[{op},
op = 0;
For[i=3, i <= 9, i++, If[i == 5, Indeterminate, op = op + (op2[[i]] - op1[[i]])^2];
Sqrt[op]
];







