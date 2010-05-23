(* ::Package:: *)

<<PlotLegends`
SetOptions[Plot, DisplayFunction-> Identity];
lmax = 0.4;
xAtImpact = 0;
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
(* Good Gains :- 1000K, 10K, 50K *)
Kp = 10000000.0;
Ki = 50000;
Kd = 10000;
iErr = 0;
iErrMax = 10000;
toSendUPhi = 0;
uPhi[t_, flagSol_] := Module[{sol, t1, t2, delT, err, derr, yDisp, thetaDisp, phiDDot},
    
    If[flagSol == 1, goal = ArcTan[xAtImpact], goal = 0];

    err = theta[t] - goal;
    derr = theta'[t];
    iErr = err + iErr;
    If[iErr >= iErrMax, iErr = iErrMax, Indeterminate];
    If[iErr <= -iErrMax, iErr = -iErrMax, Indeterminate];
    toSendUPhi = Kp*err + Kd*derr + Ki*iErr;

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
(*
, PlotLegend -> {"x", "y", "theta"}, LegendPosition-> {0.9, -0.9}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed]
*)

If[flagPhi == 0,
    pSpringParams = 
    Plot[Evaluate[{x[t], y[t], theta[t]} /. solSpring], {t, 0, end},
    PlotStyle -> {Thickness[0.005]},
    PlotLabel -> Spring Params, AxesOrigin-> {0, 0}, AxesLabel -> Automatic],

    pSpringParams = 
    Plot[Evaluate[{x[t], y[t], theta[t], phi[t]} /. solSpring], {t, 0, end}, PlotLegend -> {"x", "y", "theta", "phi"},
    LegendPosition-> {0.9, -0.9}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic,
    GridLinesStyle -> Directive[Dashed], PlotLabel -> Spring Params, AxesOrigin-> {0, 0}, AxesLabel -> Automatic]
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
(*
,PlotLegend -> {"l", "theta", "phi"},
    LegendPosition-> {0.9, -0.9}, PlotStyle -> {Thickness[0.005]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed]
    *)

If[flagPhi == 0,
    pFlightParams = 
    Plot[Evaluate[{l[t], theta[t]} /. solFlight], {t, 0, end}, PlotLegend -> {"l", "theta"},
    LegendPosition-> {0.9, -0.9},PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> Flight Params, AxesOrigin-> {0, 0}, AxesLabel -> Automatic],

    pFlightParams = 
    Plot[Evaluate[{l[t], theta[t], phi[t]} /. solFlight], {t, 0, end}, PlotStyle -> {Thickness[0.005]},
    PlotLabel -> Flight Params, AxesOrigin-> {0, 0}, AxesLabel -> Automatic]
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
(*
    PlotLegend -> {"l", "theta", "phi"}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],LegendPosition-> {0.9, -0.9},
     
    *)
If[flagPhi == 0, 
    pStanceParams = 
    Plot[Evaluate[{xFuncTemp[t], yFuncTemp[t], l[t], theta[t]}/. solStance], {t, 0, end}, PlotLegend -> {"x", "y", "l", "theta"},
    LegendPosition-> {0.9, -0.9}, PlotStyle -> {Thickness[0.003]}, Gridlines -> Automatic, GridLinesStyle -> Directive[Dashed],
    PlotLabel -> StanceParams, AxesOrigin-> {0, 0}, AxesLabel -> Automatic],

    pStanceParams = 
    Plot[Evaluate[{l[t], theta[t]}/. solStance], {t, 0, end},
    PlotStyle -> {Thickness[0.005]},
    PlotLabel -> StanceParams, AxesOrigin-> {0, 0}, AxesLabel -> Automatic]
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

lmax = 0.37;
init = {0, 0, 1, l0, 1, 0, 0, 0, 0, -1.0, 0} /. values;
flagPhi = 1;
{op1, pFlight1, pFlightParams1, solFlight1} = flightPhase[init];
{op2, pSpring1, pSpringParams1, solSpring1} = springPhase[op1];
xAtImpact = op2[[2]];
{op3, pStance1, pStanceParams1, solStance1} = stancePhase[op2];
{op4, pFlight2, pFlightParams2, solFlight2} = flightPhase[op3];
{op5, pSpring2, pSpringParams2, solSpring2} = springPhase[op4];
xAtImpact = op5[[2]];
{op6, pStance2, pStanceParams2, solStance2} = stancePhase[op5];

p = Show[pFlight1, pSpring1, pStance1, pFlight2, pSpring2, pStance2, PlotRange -> {{-0.2, 0.2},{0.1, 1.3}}];
Export["plot.pdf", p];
Export["p1.pdf", pFlightParams1];
Export["p2.pdf", pStanceParams1];
(*
tendStance = First[theta /. solStanceG][[1]][[-1]];
p1 = ParametricPlot[{Evaluate[{xFuncTemp[t], yFuncTemp[t]} /. solStanceG], Evaluate[{xFuncTemp[t], yFuncTemp[t]} /. solStance1]}
, {t, 0, tendStance}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]}, GridLines -> Automatic, 
GridLinesStyle -> Directive[Dashed]]; 

tendFlight = First[x /. solFlightG][[1]][[-1]];
p2 = ParametricPlot[{Evaluate[{x[t-tendStance], y[t-tendStance]} /. solFlightG], Evaluate[{x[t-tendStance], y[t-tendStance]}
/. solFlight2]}, {t, tendStance, tendFlight+tendStance}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]},
GridLines -> Automatic, GridLinesStyle -> Directive[Dashed]]; 

tendSpring = First[x /. solSpringG][[1]][[-1]];
p3 = ParametricPlot[{Evaluate[{x[t-tendFlight-tendStance], y[t-tendFlight-tendStance]} /. solSpringG], Evaluate[{x[t-tendFlight-tendStance], y[t-tendFlight-tendStance]} /. solSpring2]}
, {t, tendFlight + tendStance, tendStance + tendFlight + tendSpring}, AxesLabel -> {x, y}, PlotStyle -> {Thickness[0.003]},
GridLines -> Automatic, GridLinesStyle -> Directive[Dashed]]; 

p = Show[p1, p2, p3, PlotRange -> {{0, 3.5},{0,2.2}}]; 
Export["p1.pdf", p];
*)

Exit[]
