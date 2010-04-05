(* ::Package:: *)

<<PlotLegends`
SetOptions[Plot, DisplayFunction-> Identity];
lmax = 0.4;
values = {M -> 4.7, mw -> 1.5, mp -> 2.5, ml -> 0.7, Jw -> 0.02, Jb -> 0.086, K -> 500, l0 -> 0.2, h -> 0.3,
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
L = T - V;

uL[t_] =
Piecewise[{{0, l[t] <= (l0 - epsilon)},{ extendAccel, l0 <= l[t] <= ((lmax + l0)/2 + epsilon)},
{ -extendAccel, (lmax + l0)/2 < l[t] <= lmax}, {0, l[t] >  (lmax + epsilon)}} /. values];

eqConstraintPhi = phi -> Function[t, 0];
flagPhi = 0;
(* Good Gains :- 900K, 10K, 50K *)
Kp = 9000000.0;
Ki = 50000;
Kd = 10000;
iErr = 0;
iErrMax = 1;
toSendUPhi = 0;
oldToSendUPhi = 0;
uPhi[t_, flagSol_] := Module[{sol, t1},

    If[flagSol == 1, sol = solStanceG, Indeterminate];
    If[flagSol == 2, sol = solFlightG, Indeterminate];
    If[flagSol == 3, sol = solSpringG, Indeterminate];

    t1 = First[theta /. sol][[1]][[-1]];
    err = theta[t] - Evaluate[theta[t] /. sol];
    derr = theta'[t] - Evaluate[theta'[t] /. sol];
    iErr = err + iErr;
    If[iErr >= iErrMax, iErr = 0, Indeterminate];
    If[iErr <= -iErrMax, iErr = 0, Indeterminate];
    toSendUPhi = Kp*err + Ki*iErr + Kd*derr;
    toSendUPhi
];


(* Functions start here *)

springPhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=
Module[{pSpring, pSpringParams, solSpring},
start = 0;

eqConstraintSpring = l -> Function[t, len0];
eq1 = D[D[L, D[x[t], t]], t] == D[L, x[t]] /. eqConstraintSpring;
eq2 = D[D[L, D[y[t], t]], t] == D[L, y[t]] /. eqConstraintSpring;
eq3 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintSpring;
If[flagPhi == 0,
    initSpring = {x[0] == x0, y[0] == y0, theta[0] == theta0, x'[0] == xdot0, y'[0] == ydot0, 
    theta'[0] == thetadot0};
    solveForSpring = {x, y, theta};
    eq1 = eq1 /. eqConstraintPhi;
    eq2 = eq2 /. eqConstraintPhi;
    eq3 = eq3 /. eqConstraintPhi;
    eqSpring = {eq1, eq2, eq3};
    eqSpring = eqSpring /. values,

    initSpring = {x[0] == x0, y[0] == y0, theta[0] == theta0, phi[0] == phi0, x'[0] == xdot0, y'[0] == ydot0, 
    theta'[0] == thetadot0, phi'[0] == phidot0};
    solveForSpring = {x, y, theta, phi};
    eq4 = phi''[t] == uPhi[t, 2];
    eqSpring = {eq1, eq2, eq3, eq4} /. values;
];

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

If[flagPhi == 0,
    pSpringParams = 
    Plot[Evaluate[{x[t], y[t], theta[t]} /. solSpring], {t, 0, end}, PlotLegend -> {x, y, theta},LegendPosition-> {0.8, -0.8},
    PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> Spring Params, AxesOrigin-> {0, 0}],

    pSpringParams = 
    Plot[Evaluate[{x[t], y[t], theta[t], phi[t]} /. solSpring], {t, 0, end}, PlotLegend -> {x, y, theta, phi},
    LegendPosition-> {0.8, -0.8}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic,
    GridLinesStyle -> Directive[Dashed], PlotLabel -> Spring Params, AxesOrigin-> {0, 0}]
];

If[flagPhi == 0,
    toSend = {{end + t0, x[end], y[end], len0, theta[end], 0, x'[end], 
    y'[end], 0, theta'[end], 0} /. values /. solSpring, pSpring, pSpringParams, solSpring},

    toSend = {{end + t0, x[end], y[end], len0, theta[end], phi[end], x'[end], 
    y'[end], 0, theta'[end], phi'[end]} /. values /. solSpring, pSpring, pSpringParams, solSpring}
];

toSend
];


flightPhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=
Module[{pFlight, pFlightParams, solFlight},

eq1 = D[D[L, D[x[t], t]], t]== D[L, x[t]];
eq2 = D[D[L, D[y[t], t]], t]== D[L, y[t]];
eq3 = D[D[l[t], t],t] == uL[t];
eq4 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]];

If[flagPhi == 0,
    initFlight = { x[0] == x0, y[0] == y0, l[0] == len0, 
    theta[0] == theta0, x'[0] == xdot0, 
    y'[0] == ydot0, l'[0] == lendot0, theta'[0] == thetadot0};
    solveForFlight = {x, y, l, theta};
    eq1 = eq1 /. eqConstraintPhi;
    eq2 = eq2 /. eqConstraintPhi;
    eq3 = eq3 /. eqConstraintPhi;
    eq4 = eq4 /. eqConstraintPhi;
    eqFlight = {eq1, eq2, eq3, eq4} /. values,

    initFlight = {x[0] == x0, y[0] == y0, l[0] == len0, 
    theta[0] == theta0, phi[0] == phi0, x'[0] == xdot0, 
    y'[0] == ydot0, l'[0] == lendot0, theta'[0] == thetadot0, phi'[0] == phidot0};
    solveForFlight = {x, y, l, theta, phi};
    eq5 = phi''[t] == uPhi[t, 3];
    eqFlight = {eq1, eq2, eq3, eq4, eq5} /. values;
];

start = 0;
solFlight = 
First[NDSolve[Join[eqFlight, initFlight], solveForFlight, {t, 0, 5}, MaxSteps -> 10^5, 
Method -> {"EventLocator", "Event" -> (l[t] - lmax) /. values, 
"EventAction" :> Throw[end = t, "StopIntegration"]}]];

pFlight = 
ParametricPlot[
Evaluate[{x[t] /. solFlight, y[t] /. solFlight }], {t, 0, end}, 
AxesLabel -> {x, y}, PlotStyle -> {Green, Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]];

If[flagPhi == 0,
    pFlightParams = 
    Plot[Evaluate[{x[t], y[t], l[t], theta[t]} /. solFlight], {t, 0, end}, PlotLegend -> {x, y, l, theta},
    LegendPosition-> {0.8, -0.8},PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> Flight Params, AxesOrigin-> {0, 0}],

    pFlightParams = 
    Plot[Evaluate[{x[t], y[t], l[t], theta[t], phi[t]} /. solFlight], {t, 0, end}, PlotLegend -> {x, y, l, theta, phi},
    LegendPosition-> {0.8, -0.8},PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> Flight Params, AxesOrigin-> {0, 0}]
];

If[flagPhi == 0,
    toSend = {{end + t0, x[end], y[end], l[end], theta[end], 0, x'[end], 
    y'[end], 0, theta'[end], 0} /. values /. solFlight, pFlight, pFlightParams, solFlight},

    toSend = {{end + t0, x[end], y[end], l[end], theta[end], phi'[end], x'[end], 
    y'[end], 0, theta'[end], phi'[end]} /. values /. solFlight, pFlight, pFlightParams, solFlight}
];

toSend
];

momentArm[len0_, theta0_] :=
Module[{arm},
arm = d Cos[theta0] + (len - len0 - d Tan[theta0]) Sin[theta0]  /. values;
arm
];

yFuncTemp[t_] := ((len - l[t])Cos[theta[t]] - d*Sin[theta[t]] /. values);
xFuncTemp[t_] := xfootOld - (len - l[t]) Sin[theta[t]] - d Cos[theta[t]] /. values;
ydotFuncTemp[t_] := ((-len Sin[theta[t]] + l[t] Sin[theta[t]] + d Cos[theta[t]])theta'[t]
- l'[t] Cos[theta[t]] ) /. values;
xdotFuncTemp[t_] := -(len Cos[theta[t]] - l[t] Cos[theta[t]] - d Sin[theta[t]])theta'[t] + l'[t] Sin[theta[t]] /. values;

stancePhase[{t0_, x0_, y0_, len0_, theta0_, phi0_, xdot0_, ydot0_, 
lendot0_, thetadot0_, phidot0_}] :=

Module[{xdotnew, ydotnew, thetadotnew, arm, vnew, pStance, pStanceParams, solStance},
start = 0;
arm = momentArm[len0, theta0];
thetadotnew = thetadot0 - ((M*ydot0*arm/Jb ) /. values);
(*
Print["ydot stance : ", ydot0];
Print["thetadot old : ", thetadot0];
Print["thetadot new : ", thetadotnew];
*)

xfootOld = x0 + ((len - len0) Sin[theta0] + d Cos[theta0]) /. values;
eqConstraintStanceY = y ->  Function[t, (len - l[t])*Cos[theta[t]] - d*Sin[theta[t]] /. values];
eqConstraintStanceX = x -> Function[t, xfootOld - ((len - l[t]) Sin[theta[t]] + d Cos[theta[t]]) /. values];
eq1 = D[D[L, D[l[t], t]], t] == D[L, l[t]] /. eqConstraintStanceX /. eqConstraintStanceY;
eq2 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintStanceX /. eqConstraintStanceY;

If[flagPhi == 0, 
    initStance = {l[0] == len0, theta[0] == theta0, l'[0] == 0, theta'[0] == thetadotnew};
    solveForStance = {l, theta};
    eqStance = {eq1, eq2} /. eqConstraintPhi /. values,

    initStance = {l[0] == len0, theta[0] == theta0, phi[0] == phi0,
    l'[0] == 0, theta'[0] == thetadotnew, phi'[0] == phidot0};
    eq3 = phi''[t] == uPhi[t, 1];
    solveForStance = {l, theta, phi};
    eqStance = {eq1, eq2, eq3} /. values
];

solStance = 
First[NDSolve[Join[eqStance, initStance], solveForStance, {t, 0, 2}, 
Method -> {"EventLocator", "Event" -> l[t] - 0.2, 
"EventAction" :> Throw[end = t, "StopIntegration"]}]];

pStance = 
ParametricPlot[Evaluate[{xFuncTemp[t], yFuncTemp[t]}/.solStance], {t, 0, end}, 
AxesLabel -> {x, y}, PlotStyle -> {Red, Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]];

If[flagPhi == 0, 
    pStanceParams = 
    Plot[Evaluate[{xFuncTemp[t], yFuncTemp[t], l[t], theta[t]}/. solStance], {t, 0, end}, PlotLegend -> {x, y, l, theta},
    LegendPosition-> {0.8, -0.8}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> StanceParams, AxesOrigin-> {0, 0}],

    pStanceParams = 
    Plot[Evaluate[{xFuncTemp[t], yFuncTemp[t], l[t], theta[t], phi[t]}/. solStance], {t, 0, end}, PlotLegend -> {x, y, l, theta, phi},
    LegendPosition-> {0.8, -0.8}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> StanceParams, AxesOrigin-> {0, 0}]
];

vnew = ((mw + mp)*Abs[l'[end]]/M ) /. values /. solStance;
dist = Sqrt[(len - l[end])^2 + d^2 /. values /. solStance];
xdotnew =  dist*theta'[end]*Cos[theta[end]] - vnew * Sin[theta[end]] /. solStance;
ydotnew =  dist*theta'[end]*Sin[theta[end]] + vnew* Cos[theta[end]] /. solStance;

If[flagPhi == 0, 
    toSend = {{end + t0, xFuncTemp[end], yFuncTemp[end], l[end], theta[end], 0, xdotnew,
    ydotnew, 0, theta'[end], 0} /. values /. solStance, pStance, pStanceParams, solStance},

    toSend = {{end + t0, xFuncTemp[end], yFuncTemp[end], l[end], theta[end], phi[end], xdotnew,
    ydotnew, 0, theta'[end], phi'[end]} /. values /. solStance, pStance, pStanceParams, solStance}
];

toSend
];



(*------------------------------------------------------*)
(*------------------------------------------------------*)
(*------------------------------------------------------*)

getGVals[] := Module[{},
lmax = 0.4;
init = {0, 0, 1, l0, 0, 0, 1, 0, 0, -0.5, 0} /. values;
flagPhi = 0;
{op1, pFlight1, pFlightParams1, solFlight1} = flightPhase[init];
{op2, pSpring1, pSpringParams1, solSpring1} = springPhase[op1];

{op3, pStanceG, pStanceParamsG, solStanceG} = stancePhase[op2];
{op4, pFlightG, pFlightParamsG, solFlightG} = flightPhase[op3];
{op5, pSpringG, pSpringParamsG, solSpringG} = springPhase[op4];
];

getGVals[];

p = Show[pFlight1, pSpring1, pStanceG, pFlightG, pSpringG, PlotRange -> {{0, 4}, {0, 2}}];
Export["plot.eps", p];

op2[[5]] = op2[[5]] + 0.01;
flagPhi = 1;
{op3, pStance1, pStanceParams1, solStance1} = stancePhase[op2];
flagPhi = 0;
{op4, pFlight2, pFlightParams2, solFlight2} = flightPhase[op3];
{op5, pSpring2, pSpringParams2, solSpring2} = springPhase[op4];

tend = First[theta /. solStanceG][[1]][[-1]];
p1 = ParametricPlot[{Evaluate[{xFuncTemp[t], yFuncTemp[t]} /. solStanceG], Evaluate[{xFuncTemp[t], yFuncTemp[t]} /. solStance1]}
, {t, 0, tend-0.01}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]]; 
Export["p1.eps", p1];


tend = First[x /. solFlightG][[1]][[-1]];
p1 = ParametricPlot[{Evaluate[{x[t], y[t]} /. solFlightG], Evaluate[{x[t], y[t]} /. solFlight2]}
, {t, 0, tend-0.1}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]]; 
Export["p2.eps", p1];

tend = First[x /. solSpringG][[1]][[-1]];
p1 = ParametricPlot[{Evaluate[{x[t], y[t]} /. solSpringG], Evaluate[{x[t], y[t]} /. solSpring2]}
, {t, 0, tend-0.1}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]]; 
Export["p3.eps", p1];

Exit[]


