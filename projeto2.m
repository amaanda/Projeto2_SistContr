% Projeto2_SistContr
%Segundo projeto da disciplina sistema de controle - código relacionado a espaços de estados
%Adriele Ramos

%% ------------------------------------------- QUESTÃO 1

%% b) escolha dos dados
E  = 0.8; %coeficiente de amortecimento
wn = 2; %frequencia natural
a1 = 10*wn; %fator de afastamento
N = 15;

 %determinacao dos polos de malha fechada
pmf = roots(conv([1 2*E*wn wn^2],[1 a1*wn]))

%% c)
%determinacao do tempo de amostragem
T = (2*pi)/(wn*N*sqrtm(1 - E^2))

%discretização dos polos de malha fechada: mapeamento de polo-zero 
%z = e^st

pmfd = conv(conv([1 - e^-pmf(1)*T],[1 - e^-pmf(2)*T]),[1 - e^-pmf(3)*T])


%% ------------------------------------------- QUESTÃO 2

% a) o motor cc é controlável? (elder pagina 58)

A = [] % matriz 
B = [] % matriz
Q = [B A*B A^2*B] % matriz de controlabilidade

% Sistema tem ordem n. Se a matriz Q for de posto (número de linhas ou colunas LI) n, é controlável. rank = posto.

rankQ = rank(Q)

% b) matriz transformação de similaridade; e forma canônica controlável do motor cc.

% matriz transf de similaridade P
% Atransf = (inv(P))*A*P

% forma canônica controlável
% Mathworks: The A,B,C,D matrices are returned in controller canonical form. https://www.mathworks.com/help/signal/ref/tf2ss.html

[A,B,C,D] = tf2ss(num,den) % num e den: da TF de malha fechada. 

% c) determinação do vetores de ganhos de realimentação Kbarra e K. Autovalores de A-bK.

K = 
Kbarra = K*inv(P)

func = det(A-(B*K))
autovalres = roots(func)

% d) simulações da realimentação e análise em regime permanente

% simulink

%% ------------------------------------------- QUESTÃO 3
