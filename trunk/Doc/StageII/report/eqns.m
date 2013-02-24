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
eq1 = D[D[L, D[x[t], t]], t]== D[L, x[t]];
eq2 = D[D[L, D[y[t], t]], t]== D[L, y[t]];
eq3 = D[D[l[t], t],t] == uL[t];
eq4 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]];
eq5 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]];
eqFlight = {eq1, eq2, eq3, eq4, eq5} // Simplify;

xfootOld = x0 + ((len - len0) Sin[theta0] + d Cos[theta0]);
eqConstraintStanceY = y ->  Function[t, (len - l[t])*Cos[theta[t]] - d*Sin[theta[t]]];
eqConstraintStanceX = x -> Function[t, xfootOld - ((len - l[t]) Sin[theta[t]] + d Cos[theta[t]])];
eq1 = D[D[L, D[l[t], t]], t] == D[L, l[t]] /. eqConstraintStanceX /. eqConstraintStanceY;
eq2 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintStanceX /. eqConstraintStanceY;
eq3 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]] /. eqConstraintStanceX /. eqConstraintStanceY;
eqStance = {eq1, eq2, eq3} // Simplify;

eqConstraintSpring = l -> Function[t, len0];
eq1 = D[D[L, D[x[t], t]], t] == D[L, x[t]] /. eqConstraintSpring;
eq2 = D[D[L, D[y[t], t]], t] == D[L, y[t]] /. eqConstraintSpring;
eq3 = D[D[L, D[theta[t], t]], t] == D[L, theta[t]] /. eqConstraintSpring;
eq4 = D[D[L, D[phi[t], t]], t] == D[L, phi[t]] /. eqConstraintSpring;
eqSpring = {eq1, eq2, eq3, eq4} // Simplify;

