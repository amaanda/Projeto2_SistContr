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

%% -----------------------------------------QUESTÃO 1
% b) escolha dos dados
E  = 0.8; %coeficiente de amortecimento
wn = 2; %frequencia natural
a1 = 10*wn; %fator de afastamento
N = 15;

 %determinacao dos polos de malha fechada
pmf = roots(conv([1 2*E*wn wn^2],[1 a1*wn]))


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

%polinômeio característico da dinâmica do motor cc
[num,den] = ss2tf(A,B,C,D) %equação característica a partir das matrizes de ss - retorna coeficientes do numerador e denominador



%% ------------------------------------------- QUESTÃO 2

% a) o motor cc é controlável? (elder pagina 58)

A = [] % matriz 
B = [] % matriz
Co = ctrb(A,B); % matriz de controlabilidade

% Sistema tem ordem n. Se a matriz Q for de posto (número de linhas ou colunas LI) n, é controlável. rank = posto.

rankCo = rank(Co);

% b) matriz transformação de similaridade; e forma canônica controlável do motor cc.

% matriz transf de similaridade P
% elder pg 104, kuo pg 118 pra frente

q3 = B;
q2 = A*q3 + 3*q3;
q1 = A*q2 + q3;
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

Ob = obsv(A,C);
rankOb = rank(Ob);

% b) escolha do fator de rapidez do observador

a0 = 4; 

% e pólos do observador

Ob_pmf = 

% Discretização dos pólos do observador e determinação do polinômio característico discretizado.

Ob_pmfd = exp(Ob_pmf.*T)

