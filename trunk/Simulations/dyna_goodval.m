(* ::Package:: *)

<<PlotLegends`
SetOptions[Plot, DisplayFunction-> Identity];
lmax = 0.4;
values={M -> 4.7, mw -> 1.5, mp -> 2.5, ml -> 0.7, Jw -> 0.02, Jb -> 0.086, K -> 2500, l0 -> 0.2, h -> 0.3,
d -> 0.05, dw -> 0.2, g-> 9.8, len -> 0.5, hmax -> 1, extendAccel -> 15, epsilon -> 0.01};

xl[t_]= x[t] + (d - (l[t]- h) Tan [theta[t]]) Cos [theta[t]];
yl[t_]= x[t] + (l[t] - h) Cos[theta[t]] + d Sin[theta[t]];

xw[t_]= x[t] - (dw -d) Cos[theta[t]];
yw[t_]= y[t] -(dw -d) Sin[theta[t]];

xp[t_]= x[t] + d Cos[theta[t]];
yp[t_]= y[t] + d Sin[theta[t]];


T = 1/2 mw (xw'[t]^2 + yw'[t]^2) +  1/2 mp (xp'[t]^2 + yp'[t]^2) +  1/2 ml (xl'[t]^2 + yl'[t]^2) + 1/2 Jw (phi'[t] + theta'[t])^2 + 1/2 Jb theta'[t]^2;
V = g (ml yl[t] + mw yw[t] + mp yp[t]) + 1/2 K (l[t] - l0)^2;


uL[t_] =
Piecewise[{{0, l[t] <= (l0-epsilon)},{ extendAccel, l0 <= l[t] <= ((lmax + l0)/2 + epsilon)}, { -extendAccel, (lmax+l0)/2 < l[t] <= lmax}, {0, l[t] >  (lmax+epsilon)}} /. values];

L = T - V;
eq1 = D[D[L, D[x[t], t]], t]== D[L, x[t]];
eq2 = D[D[L, D[y[t], t]], t]== D[L, y[t]];
eq3 = D[D[l[t], t],t] == uL[t];
eq4 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]];
eq5 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]];
eqFlight = {eq1, eq2, eq3, eq4, eq5};

eqFlight = eqFlight /. values;

springPhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=
Module[{pSpring, pSpringParams},
start = 0;
initSpring = { x[0] == x0, y[0] == y0, theta[0] == theta0, 
phi[0] == phi0, x'[0] == xdot0, y'[0] == ydot0, 
theta'[0] == thetadot0, phi'[0] == phidot0};
solveForSpring = {x, y, theta, phi};

eqConstraintSpring = l -> Function[t, len0];
L = T - V;
L = L /. eqConstraintSpring;
eq1 = D[D[L, D[x[t], t]], t] == D[L, x[t]];
eq2 = D[D[L, D[y[t], t]], t] == D[L, y[t]];
eq3 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]];
eq4 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]];
eqSpring = {eq1, eq2, eq3, eq4};
eqSpring = eqSpring /. values;

solSpring = 
First[NDSolve[Join[eqSpring, initSpring], 
solveForSpring, {t, 0, 5}, MaxSteps -> 10^8, 
Method -> {"EventLocator", "Event" -> (y[t] - (len - len0)*Cos[theta[t]] + d*Sin[theta[t]]) /. values, 
"EventAction" :> Throw[end = t, "StopIntegration"]}]];


pSpring = 
ParametricPlot[
Evaluate[{x[t] /. solSpring, y[t] /. solSpring}], {t, 0, end}, 
AxesLabel -> {x, y}, PlotStyle -> {Blue, Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]];

pSpringParams = 
Plot[Evaluate[{x[t], y[t], theta[t]} /. solSpring], {t, 0, end}, PlotLegend -> {x, y, theta},LegendPosition-> {0.8, -0.8},
PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
PlotLabel -> Spring Params, AxesOrigin-> {0, 0}];


{{end + t0, x[end], y[end], len0, theta[end], phi[end], x'[end], 
y'[end], 0, theta'[end], phi'[end]} /. values /. solSpring, 
pSpring, pSpringParams}
];


flightPhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=
Module[{pFlight, pFlightParams},

start = 0;
initFlight = { x[0] == x0, y[0] == y0, l[0] == len0, 
theta[0] == theta0, phi[0] == phi0, x'[0] == xdot0, 
y'[0] == ydot0, l'[0] == lendot0, theta'[0] == thetadot0, 
phi'[0] == phidot0};

solveForFlight = {x, y, l, theta, phi};
solFlight = 
First[NDSolve[Join[eqFlight, initFlight], 
solveForFlight, {t, 0, 5}, MaxSteps -> 10^8, 
Method -> {"EventLocator", 
"Event" -> (l[t] - lmax) /. values, 
"EventAction" :> Throw[end = t, "StopIntegration"]}]];

pFlight = 
ParametricPlot[
Evaluate[{x[t] /. solFlight, y[t] /. solFlight }], {t, 0, end}, 
AxesLabel -> {x, y}, PlotStyle -> {Green, Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]];

pFlightParams = 
Plot[Evaluate[{x[t], y[t], l[t], theta[t]} /. solFlight], {t, 0, end}, PlotLegend -> {x, y, l, theta},
LegendPosition-> {0.8, -0.8},PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
PlotLabel -> Flight Params, AxesOrigin-> {0, 0}];

{{end + t0, x[end], y[end], l[end], theta[end], phi[end], x'[end], 
y'[end], 0, theta'[end], phi'[end]} /. values /. 
solFlight, pFlight, pFlightParams}
];

momentArm[len0_, theta0_] :=
Module[{arm},
arm = d Cos[theta0] + (len - len0 - d Tan[theta0]) Sin[theta0]  /. values;
arm
];


stancePhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=

Module[{xdotnew, ydotnew, thetadotnew, arm, vnew, pStance, pStanceParams},
start = 0;
arm = momentArm[len0, theta0];
thetadotnew = thetadot0 - ((M*ydot0*arm/Jb ) /. values);
(*
Print["ydot stance : ", ydot0];
Print["thetadot old : ", thetadot0];
Print["thetadot new : ", thetadotnew];
*)

initStance = {(l[0] == len0) /. values, theta[0] == theta0, phi[0] == phi0, l'[0] == 0,
theta'[0] == thetadotnew, phi'[0] == phidot0};

xfootOld = x0 + ((len - len0) Sin[theta0] + d Cos[theta0]) /. values;
eqConstraintStanceY = y ->  Function[t, (len - l[t])*Cos[theta[t]] - d*Sin[theta[t]]];
eqConstraintStanceX = x -> Function[t, xfootOld - ((len - l[t]) Sin[theta[t]] + d Cos[theta[t]])];
L = T -V;
L = L /. eqConstraintStanceX;
L = L /. eqConstraintStanceY;
eq1 = D[D[L, D[l[t], t]], t] == D[L, l[t]];
eq2 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]];
eq3 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]];
eqStance = {eq1, eq2, eq3};
eqStance = eqStance /. values;

solveForStance = {l, theta, phi};
solStance = 
First[NDSolve[Join[eqStance, initStance], 
solveForStance, {t, 0, 2}, 
Method -> {"EventLocator", "Event" -> (l[t] - l0) /. values, 
"EventAction" :> Throw[end = t, "StopIntegration"]}]];

yFuncTemp[t_] := ((len - l[t])Cos[theta[t]] - d*Sin[theta[t]] /. values) /. solStance;
xFuncTemp[t_] := xfootOld - (len - l[t]) Sin[theta[t]] - d Cos[theta[t]] /. values /. solStance;
ydotFuncTemp[t_] := ((-len Sin[theta[t]] + l[t] Sin[theta[t]] + d Cos[theta[t]])theta'[t]
- l'[t] Cos[theta[t]] ) /. values /. solStance;
xdotFuncTemp[t_] := -(len Cos[theta[t]] - l[t] Cos[theta[t]] - d Sin[theta[t]])theta'[t] + l'[t] Sin[theta[t]] /. values /. solStance;

pStance = 
ParametricPlot[
Evaluate[{xFuncTemp[t], yFuncTemp[t] }], {t, 0, end}, 
AxesLabel -> {x, y}, PlotStyle -> {Red, Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]];

pStanceParams = 
Plot[Evaluate[{xFuncTemp[t], yFuncTemp[t], l[t] /. solStance, theta[t] /. solStance}], {t, 0, end}, PlotLegend -> {x, y, l, theta},
LegendPosition-> {0.8, -0.8}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
PlotLabel -> StanceParams, AxesOrigin-> {0, 0}];

vnew = ((mw + mp)*Abs[l'[end]]/M ) /. values /. solStance;
dist = Sqrt[(len - l[end])^2 + d^2 /. values /. solStance];
xdotnew =  dist*theta'[end]*Cos[theta[end]] - vnew * Sin[theta[end]] /. solStance;
ydotnew =  dist*theta'[end]*Sin[theta[end]] + vnew* Cos[theta[end]] /. solStance;
Print["ydotnew: ", ydotnew];

{{end + t0, xFuncTemp[end], yFuncTemp[end], l[end], theta[end], phi[end], 
xdotnew, ydotnew, 0, theta'[end], phi'[end]} /. values /. 
solStance, pStance, pStanceParams}
];


oneBounce[vTopHoriz_, hTopMax_, lengthMax_, thetaTop_, thetaDotTop_] :=
Module[{},

lmax = lengthMax;
init = {0, 0, hTopMax, l0, thetaTop, 0, vTopHoriz, 0, 0, thetaDotTop, 0} /. values;

{op1, p1Flight, p1FlightParams} = flightPhase[init];
{op2, p1Spring, p1SpringParams} = springPhase[op1];
{op3, p1Stance, p1StanceParams} = stancePhase[op2];
{op4, p2Flight, p2FlightParams} = flightPhase[op3];
{op5, p2Spring, p2SpringParams} = springPhase[op4];
{op6, p2Stance, p2StanceParams} = stancePhase[op5];
{op7, p3Flight, p3FlightParams} = flightPhase[op6];
{op8, p3Spring, p3SpringParams} = springPhase[op7];

xmax = op8[[2]] + 0.5;
ymin = -0.5;
pTotal = Show[p1Flight, p1Spring, p1Stance, p2Flight, p2Spring, p2Stance, p3Flight, p3Spring, PlotRange -> {{0, xmax},{ymin, 2}}];
Export["plot.eps", pTotal];

norm = Sqrt[(op8[[3]]-op5[[3]])^2 + (op8[[5]]-op5[[5]])^2 + (op8[[7]]-op5[[7]])^2 +
(op8[[8]]-op5[[8]])^2 + (op8[[10]]-op5[[10]])^2];

norm
];

cf = Compile[{ {vTemp, _Real}, {hTemp, _Real}, {lTemp, _Real}, {thTemp, _Real}, {thdotTemp, _Real}}, oneBounce[vTemp, hTemp,
lTemp, thTemp, thdotTemp]];



n = oneBounce[3, 1.1, 0.35, -0.15 , 0.4];
Print["norm: ", n];

(*
NMinimize[oneBounce[velHoriz, height, lengthMax, thetaTop, thetaDotTop], {{velHoriz, 2.8, 3}, {height, 1.0, 1.2},
{lengthMax, 0.25, 0.35}, {thetaTop, -1, 1}, {thetaDotTop, -3, 3}}, Method -> "DifferentialEvolution",
AccuracyGoal -> 1, PrecisionGoal -> 1, WorkingPrecision -> 2];

NMinimize[oneBounce[velHoriz, height, lengthMax, thetaTop, thetaDotTop], {{thetaDotTop, -1, 1}},
Method -> "DifferentialEvolution", AccuracyGoal -> 1, PrecisionGoal -> 1, WorkingPrecision -> 2];
*)

(*
height = 1.1;
velHoriz = 3.0;
lengthMax = 0.35;
thetaTop = 0.;


minimum = 1000;
count = 0;
For[i=-0.155, i<= -0.145, i = i+0.002, For[j=0.35, j<=0.45, j=j+0.02, n = oneBounce[velHoriz, height, lengthMax, i, j];
If[n<minimum, minimum = n; Print["n: ", n, " theta: ", i, " thetaDot: ", j]]; count++; Print[count]]];
*)

Exit[]

