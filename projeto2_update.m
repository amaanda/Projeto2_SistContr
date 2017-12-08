% Projeto2_SistContr
%Segundo projeto da disciplina sistema de controle - código relacionado a espaços de estados
%Adriele Ramos

%Parâmetros
R = 3.33;
L = 4.56*(10^-3);
j = 4.96*(10^-5);
b = 4.59*(10^-5);
K = 0.0332;

%Matrizes de equação de estado motor cc
A = [-R/L -K/L 0; K/j b/j 0; 0 1 0];
B = [1/L;0;0];
C = [0 0 1];
D = [0];

syms z;

%representação em EE
M = ss(A,B,C,D);

%% -----------------------------------------QUESTÃO 1
% b) escolha dos dados
E  = 0.8; %coeficiente de amortecimento
wn = 2; %frequencia natural
a1 = 10; %fator de afastamento
N = 15;

 %determinacao dos polos de malha fechada
pmf = roots(conv([1 2*E*wn wn^2],[1 a1*wn])) % retorna nessa ordem: polo real, polos complexos


% c)
%determinacao do tempo de amostragem
T = (2*pi)/(wn*N*sqrt(1 - E^2))

%discretização dos polos de malha fechada: mapeamento de polo-zero 
%z = e^st
pmfd = exp(pmf.*T)

%determinação do polinômio característico de malha fechada discretizado
%equação caracteristica discretizada
eqdisc = conv(conv([1 - pmfd(1)],[1 - pmfd(2)]),[1 - pmfd(3)])

%discretização da equação de estados do motor cc
sysG = ss(A,B,C,D) %matrizes equação de estado
sysGd = c2d(sysG, T) %discretização por zoh - conforme Franklin (discreto)

%polinômio característico da dinâmica do motor cc
tam = size(A);
n = tam(1,1);
[num,den] = ss2tf(A,B,C,D) %den são os coeficientes do polinomio característico

pc = roots(den) %raízes polinomio característico = POLOS DA FT
pcd = exp(pc.*T) %raizes polinomio caracteristico discretizadas
eqdiscmot = conv(conv([1 - pcd(1)],[1 - pcd(2)]),[1 - pcd(3)]) %equação discretizada 

% representação em EE do Motor Discretizado
Md = c2d(M,T,'zoh');

% Polinômio característico da dinâmica do motor CC de Ad (autovalores de A são os pólos do sistema)
pca = poly(Md.A)

%% ------------------------------------------- QUESTÃO 2

% a) o motor cc é controlável? (elder pagina 58)

%A = [] % matriz 
%B = [] % matriz
Co = ctrb(Md.A,Md.B); % matriz de controlabilidade

% Sistema tem ordem n. Se a matriz Q for de posto (número de linhas ou colunas LI) n, é controlável. rank = posto.

rankCo = rank(Co);

% b) matriz transformação de similaridade; e forma canônica controlável do motor cc.

% matriz transf de similaridade P
% elder pg 104, kuo pg 118 pra frente

% a2 e a3 são coeficientes da equação discretizada do motor!!!!!
a2 = pca(2);
a3 = pca(3);

q3 = Md.B;
q2 = Md.A*q3 + a2*q3;
q1 = Md.A*q2 + a3*q3;
Q = [q1 q2 q3];
P = inv(Q);

% forma canônica controlável
% Mathworks: The A,B,C,D matrices are returned in controller canonical form. https://www.mathworks.com/help/signal/ref/tf2ss.html

[A,B,C,D] = tf2ss(num,den); % num e den: da TF de malha fechada. 

% c) determinação do vetores de ganhos de realimentação Kbarra e K. Autovalores de A-bK.

%% pmf polos de malha fechada da questão 1
K = place(A,B,pmf);
Kbarra = K*inv(P);

func = det(A-(B*K));
autovalores = roots(func);

% d) simulações da realimentação e análise em regime permanente

% simulink

%% ------------------------------------------- QUESTÃO 3

% a) o motor é observável?

Ob = obsv(Md);
rankOb = rank(Ob);

% b) escolha do fator de rapidez do observador

a0 = 4; 

% e pólos do observador

Ob_pmf = roots(conv([1 2*E*a0*wn wn^2],[1 a1*a0*wn])) % retorna nessa ordem: polo real, polos complexos

% Discretização dos pólos do observador e determinação do polinômio característico discretizado.

Ob_pmfd = exp(Ob_pmf.*T)
Ob_eqdisc = conv(conv([1 - Ob_pmfd(1)],[1 - Ob_pmfd(2)]),[1 - Ob_pmfd(3)]);

% c) determinação dos vetores de ganhos Lbarra e L.


%polinomio = conv(conv([1 - Ob_eqdisc(1)],[1 - Ob_eqdisc(2)]),[1 - Ob_eqdisc(3)]);
%alfasbarra = transpose(roots(polinomio)); % roots retorna vetor coluna
alfasbarra = Ob_pmfd
alfas = pcd % roots retorna vetor coluna

LbarraT = [alfasbarra-alfas]
Lbarra = transpose(LbarraT)

L = inv(P)*LbarraT

% d) simulações da realimentação e análise em regime permanente

% simulink

%% --------------------------------QUESTÃO 4
% observador de ordem (n-1)

%a) 
%escolha do fator de rapidez do observador
%pólos do observador
%MESMO FATOS E MESMOS POLOS DO OBSERVADOR DA QUESTÃO 3

%b)
%discretização dos polos do observador
%determinação do polinômio característico discretizado
%IGUAL QUESTÃO 3

%c)
%matriz de transformação de similaridade 

P = []

%equação do observador


%% ------------------------------------------- QUESTÃO 5


%% ------------------------------------------- QUESTÃO 6
