(* ::Package:: *)

<<PlotLegends`
SetOptions[Plot, DisplayFunction-> Identity];
lmax = 0.4;
values={M -> 4.7, mw -> 1.5, mp -> 2.5, ml -> 0.7, Jw -> 0.02, Jb -> 0.086, K -> 500, l0 -> 0.2, h -> 0.3,
d -> 0.05, dw -> 0.2, g-> 9.8, len -> 0.5, hmax -> 1, extendAccel -> 15, epsilon -> 0.01};

xl[t_]= x[t] + (d - (l[t]- h) Tan [theta[t]]) Cos [theta[t]];
yl[t_]= x[t] + (l[t] - h) Cos[theta[t]] + d Sin[theta[t]];

xw[t_]= x[t] - (dw -d) Cos[theta[t]];
yw[t_]= y[t] -(dw -d) Sin[theta[t]];

xp[t_]= x[t] + d Cos[theta[t]];
yp[t_]= y[t] + d Sin[theta[t]];
T = 1/2 mw (xw'[t]^2 + yw'[t]^2) +  1/2 mp (xp'[t]^2 + yp'[t]^2) +  1/2 ml (xl'[t]^2 + yl'[t]^2) +
1/2 Jw (phi'[t] + theta'[t])^2 + 1/2 Jb theta'[t]^2;
V = g (ml yl[t] + mw yw[t] + mp yp[t]) + 1/2 K (l[t] - l0)^2;

uL[t_] =
Piecewise[{{0, l[t] <= (l0 - epsilon)},{ extendAccel, l0 <= l[t] <= ((lmax + l0)/2 + epsilon)},
{ -extendAccel, (lmax + l0)/2 < l[t] <= lmax}, {0, l[t] >  (lmax + epsilon)}} /. values];

eqConstraintPhi = phi -> Function[t, 0];

L = T - V;
eq1 = D[D[L, D[x[t], t]], t]== D[L, x[t]] /. eqConstraintPhi;
eq2 = D[D[L, D[y[t], t]], t]== D[L, y[t]] /. eqConstraintPhi;
eq3 = D[D[l[t], t],t] == uL[t] /. eqConstraintPhi;
eq4 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintPhi;
eqFlight = {eq1, eq2, eq3, eq4};
eqFlight = eqFlight /. values;

(* Functions start here *)

springPhase[{t0_, x0_, y0_, len0_, theta0_, xdot0_, ydot0_, 
lendot0_, thetadot0_}] :=
Module[{pSpring, pSpringParams},
start = 0;
initSpring = {x[0] == x0, y[0] == y0, theta[0] == theta0, x'[0] == xdot0, y'[0] == ydot0, 
theta'[0] == thetadot0};
solveForSpring = {x, y, theta};

eqConstraintSpring = l -> Function[t, len0];
eq1 = D[D[L, D[x[t], t]], t] == D[L, x[t]] /. eqConstraintSpring /. eqConstraintPhi;
eq2 = D[D[L, D[y[t], t]], t] == D[L, y[t]] /. eqConstraintSpring /. eqConstraintPhi;
eq3 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintSpring /. eqConstraintPhi;
eqSpring = {eq1, eq2, eq3};
eqSpring = eqSpring /. values;

solSpring = 
First[NDSolve[Join[eqSpring, initSpring], 
solveForSpring, {t, 0, 5}, MaxSteps -> 10^6, 
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

{{end + t0, x[end], y[end], len0, theta[end], x'[end], 
y'[end], 0, theta'[end]} /. values /. solSpring, 
pSpring, pSpringParams}
];
flightPhase[{t0_, x0_, y0_, len0_, theta0_, xdot0_, ydot0_, 
lendot0_, thetadot0_}] :=
Module[{pFlight, pFlightParams},

start = 0;
initFlight = { x[0] == x0, y[0] == y0, l[0] == len0, 
theta[0] == theta0, x'[0] == xdot0, 
y'[0] == ydot0, l'[0] == lendot0, theta'[0] == thetadot0};

solveForFlight = {x, y, l, theta};
solFlight = 
First[NDSolve[Join[eqFlight, initFlight], 
solveForFlight, {t, 0, 5}, MaxSteps -> 10^6, 
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

{{end + t0, x[end], y[end], l[end], theta[end], x'[end], 
y'[end], 0, theta'[end]} /. values /. solFlight, pFlight, pFlightParams}
];

momentArm[len0_, theta0_] :=
Module[{arm},
arm = d Cos[theta0] + (len - len0 - d Tan[theta0]) Sin[theta0]  /. values;
arm
];
stancePhase[{t0_, x0_, y0_, len0_, theta0_, xdot0_, ydot0_, 
lendot0_, thetadot0_}] :=

Module[{xdotnew, ydotnew, thetadotnew, arm, vnew, pStance, pStanceParams},
start = 0;
arm = momentArm[len0, theta0];
thetadotnew = thetadot0 - ((M*ydot0*arm/Jb ) /. values);
(*
Print["ydot stance : ", ydot0];
Print["thetadot old : ", thetadot0];
Print["thetadot new : ", thetadotnew];
*)

initStance = {(l[0] == len0) /. values, theta[0] == theta0, l'[0] == 0, theta'[0] == thetadotnew};

xfootOld = x0 + ((len - len0) Sin[theta0] + d Cos[theta0]) /. values;
eqConstraintStanceY = y ->  Function[t, (len - l[t])*Cos[theta[t]] - d*Sin[theta[t]]];
eqConstraintStanceX = x -> Function[t, xfootOld - ((len - l[t]) Sin[theta[t]] + d Cos[theta[t]])];
eq1 = D[D[L, D[l[t], t]], t] == D[L, l[t]] /. eqConstraintStanceX /. eqConstraintStanceY /. eqConstraintPhi;
eq2 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintStanceX /. eqConstraintStanceY /. eqConstraintPhi;
eqStance = {eq1, eq2};
eqStance = eqStance /. values;

solveForStance = {l, theta};
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

{{end + t0, xFuncTemp[end], yFuncTemp[end], l[end], theta[end], xdotnew,
ydotnew, 0, theta'[end]} /. values /. solStance, pStance, pStanceParams}
];


oneBounce[vTopHoriz_, hTopMax_, lengthMax_, thetaTop_, thetaDotTop_] :=
Module[{},

lmax = lengthMax;
init = {0, 0, hTopMax, l0, thetaTop, vTopHoriz, 0, 0, thetaDotTop} /. values;

{op1, p1Flight, p1FlightParams} = flightPhase[init];
{op2, p1Spring, p1SpringParams} = springPhase[op1];
{op3, p1Stance, p1StanceParams} = stancePhase[op2];
{op4, p2Flight, p2FlightParams} = flightPhase[op3];
{op5, p2Spring, p2SpringParams} = springPhase[op4];
{op6, p2Stance, p2StanceParams} = stancePhase[op5];
{op7, p3Flight, p3FlightParams} = flightPhase[op6];
{op8, p3Spring, p3SpringParams} = springPhase[op7];

pTotal = Show[p1Flight, p1Spring, p1Stance, p2Flight, p2Spring, p2Stance, p3Flight, p3Spring, PlotRange -> {{0, 4},{0, 2}}];
norm = Sqrt[(op8[[3]]-op5[[3]])^2 + (op8[[5]]-op5[[5]])^2 + (op8[[6]]-op5[[6]])^2 +
(op8[[7]]-op5[[7]])^2 + (op8[[9]]-op5[[9]])^2];
Print["norm:", norm];

Export["plot.eps", pTotal];
norm

];

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

cost[op1_List, op2_List] := Module[{},
output = 0;
For[i=3, i <= 9, i++, output = output + (op2[[i]] - op1[[i]])^2];
Sqrt[output]
];

forOpti[th_Real, dth_Real, opStart_List] := Module[{op1},
    opStart[[5]] = th;
    opStart[[9]] = dth;
    op1 = poincareMap[opStart];
    c = cost[op1, opStart];
    c
];

init = {0, 0, 1.0, l0, 0, 1.0, 0, 0, -0.5} /. values;
{op1, pt1, pt2} = flightPhase[init];
{op2, pt1, pt2} = springPhase[op1];
op2[[1]] = 0;
op2[[2]] = 0;

op1 = poincareMap[op2];
Print[op1];
op2 = poincareMap[op1];
Print[op2];
Print[cost[op1, op2]];
Print["Iterating near this value now..."];

th = op1[[5]];
thMin = op1[[5]] - 0.01;
dthMin = op1[[9]] - 0.01;
thMax = op1[[5]] + 0.01;
dthMax = op1[[9]] + 0.01;

For[dth=dthMin, dth<= dthMax, dth = dth + 0.001,
    a = forOpti[th, dth, op1];
    Print[{th, dth, a}];
];

(*
        NMinimize[forOpti[th, dth, op1], {{th, thMin, thMax}, {dth, dthMin, dthMax}}, Method -> "Automatic",
StepMonitor -> Print[{th, dth}]];
*)
(*
For[th = -0.09, th <= 0.1, th = th + 0.01,
    For[dth = -0.09, dth <= 0.1, dth = dth + 0.01,
        init = {0, 0, 1.0, l0, 0, 3.0, 0, 0, dth} /. values;
        {op1, pt1, pt2} = flightPhase[init];
        {op2, pt1, pt2} = springPhase[op1];
        op2[[1]] = 0;
        op2[[2]] = 0;

        op = poincareMap[op2];
        (*Print[op];*)
        op1 = poincareMap[op];
        (*Print[op1];*)
        a = cost[];
        OutputForm[Print["th=",0," dth=",dth,"cost=",a]];
    ]
];
*)


Exit[]







