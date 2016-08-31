clear all;

% Precision 

Acc= 15;

% Discretisation de la fonction de transfert

Te=0.01;
Ks=0.25171;
tau=0.0314;
Gs=tf([Ks],[tau 1 0]);
Gz=c2d(Gs,Te,'zoh');
[B,A]=tfdata(Gz,'v');
K=B(2);
z1=roots(B);
poles_A=roots(A);
p1=poles_A(2);

% Definition des pôles et zéros à compenser

syms Bx B_ Ax A_ z_;
d=2;
Bx=1;
B_=K*(1-z1*z_);
Ax=(1-p1*z_);
A_=(1-z_);

% Definition du modèle de suivi de consigne

wn=40;
zeta=0.707;
alpha=exp(-zeta*wn*Te);
beta=wn*Te*sqrt(1-zeta^2);

syms Am;

Am=vpa(1-2*alpha*cos(beta)*z_+alpha^2*z_^2,Acc);

% Obectif supplémentaire : suivi d'une rampe

syms b0 b1 b2 p0 p1 p2;

Bmx=b0+b1*z_+b2*z_^2;
B0=p0+p1*z_+p2*z_^2;

OS=vpa(coeffs(collect(expand(z_^d*B_*Bmx+(1-z_)^3*B0)),z_),Acc);
SM=coeffs(Am,z_);

eq1=OS(1)-SM(1);
eq2=OS(2)-SM(2);
eq3=OS(3)-SM(3);
eq4=OS(4);
eq5=OS(5);
eq6=OS(6);

[b0 b1 b2 p0 p1 p2]=solve(eq1,eq2,eq3,eq4,eq5,eq6);

b0=b0(1);
b1=b1(1);
p0=p0(1);
p1=p1(1);

M=vpa(z_*B_*Bmx/Am,Acc);

% Ajout d'integrateur pour le rejet de perturbation

p=1;

% Choix d'un filtre supplémentaire

A0=1-0.96*z_;

% Résolution des équations diophantiennes

syms s0 s1 s2 r0 r1;

S2=s0+s1*z_+s2*z_^2;
R0=r0+r1*z_;

ED1=(1-z_)^p*A_*S2+z_^d*B_*R0;
CED1=vpa(coeffs(collect(expand(ED1)),z_),Acc);
EDSM1=A0*Am;
CEDSM1=vpa(coeffs(collect(expand(EDSM1)),z_),Acc);

eq1=CED1(1)-CEDSM1(1);
eq2=CED1(2)-CEDSM1(2);
eq3=CED1(3)-CEDSM1(3);
eq4=CED1(4)-CEDSM1(4);
eq5=CED1(5);

[r0 r1 s0 s1 s2]=solve(eq1,eq2,eq3,eq4,eq5);

r0=r0(1);
r1=r1(1);
s0=s0(1);
s1=s1(1);
s2=s2(1);

T0=Bmx*A0;

% Calcul du correcteur RST

format long e;
R=double(vpa(coeffs(collect(expand(eval(Ax*R0))),z_),Acc));
S=double(vpa(coeffs(collect(expand(eval((1-z_)^p*Bx*S2))),z_),Acc));
T=double(vpa(coeffs(collect(expand(eval(Ax*T0))),z_),Acc));




