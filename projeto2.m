% Projeto2_SistContr
%Segundo projeto da disciplina sistema de controle - código relacionado a espaços de estados
%Adriele Ramos

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

