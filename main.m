clear all
close all
clc

%% Progetto Teoria dei Sistemi 2021-2022
% Controllo del beccheggio (pitch) di un aeromobile in volo stazionario
% Docente: Prof. Gianluca Antonelli
% Alunno: Russo Giulio Mat. 0051267

%% Modello Lineare
%{

    Il modello fa riferimento a un moto longitudinale del velivolo in
    condizione di volo stazionario (rettilineo con velocità e altitudine
    costanti)

    Le equazioni sono un set di 6 espressioni differenziali non lineari con:

    teta = angolo di imbardata, per il controllo della rotta del velivolo.
    Ottenuto tramite il movimento del timone, provoca una asimmetria nella
    distribuzione di portanza per cui la semiala esterna con velocità angolare
    maggiore provoca un rollio della fusoliera verso l'interno della virata,
    che il pilota deve opportunamente bilanciare

    q = derivata dell'imbardata ~ rate di imbardata (velocità)

    alpha = angolo di attacco, l'angolo tra una linea di riferimento su un corpo
    (spesso la linea di corda di un profilo aerodinamico) e il vettore che
    rappresenta il moto relativo tra il corpo e il fluido attraverso il quale è
    in movimento

    delta = angolo di deflessione. Si definisce una verticale geodetica (retta
    ortogonale al piano tangente alla superficie nel punto considerato) e una
    verticale astronomica (il filo a piombo). L'angolo tra queste due verticali
    è chiamato angolo di deflessione. Ed è il nostro ingresso

    mu = coefficiente che tiene conto della densità dell'aria, superficie alare,
    lunghezza della corda alare e della massa dell'aeroplano

    omega = coefficiente che tiene conto della velocità di volo all'equilibrio e
    della corda alare

    CD = resistenza dell'aria

    CL = coefficiente di portanza

    CW = coefficiente di peso

    CM = coefficiente del momento di beccheggio

    gamma = angolo della rotta di volo (teta - alpha)

    iyy = momento di inerzia

    sigma = diepnde da mu e CL

    eta = dipende da mu, sigma e CM



    Utilizzando dati sperimentali otteniamo il modello:

    dalpha   -0.313   56.7    0   alpha   0.232
    dq     = -0.0139  -0.426  0 * q     + 0.0203 * delta
    dteta     0       56.7    0   teta    0

                alpha
    y = 0 0 1 * q     + 0 * delta
                teta

%}

% Matrice dinamica
A=[ 
    -0.313      56.7       0
    -0.0139    -0.426      0
     0          56.7       0
   ];

% Matrice di guadagno
B=[ 
    0.232
    0.0203
    0
   ];

% Matrice di uscita
C=[0 0 1]; % misuro teta

% Matrice di legame
D=0;

% Condizioni iniziali
deg=15;
x0=[0 0 deg*pi/180]'; % [alpha q teta]
%{
    Impongo su teta una condizione iniziale di 15 gradi, pari ad una
    inclinazione di decollo media
%}

% Rumore
odg_w=1e-4;
%{
    Ordine di grandezza del rumore gaussiano bianco sull'uscita
%}





%% Questito 1 - Caratterizzazione in termini di Autovalori, Costanti di tempo, Raggiungibilità e Osservabilità

fprintf("\n Autovalori della matrice dinamica \n");
% Autovalori della matrice dinamica
eigA=eig(A)
%{
     0.0000 + 0.0000i
    -0.3695 + 0.8860i
    -0.3695 - 0.8860i

    >> eig(A)=roots(poly(A))

    Analizziamo gli autovalori della matrice dinamica:
    Il sistema ha tre autovalori. Un autovalore nullo associato ad un modo
    naturale costante, e due autovalori complessi coniugati a parte reale
    minore di zero associati a due modi naturali sinusoidali contenuti da un
    esponenziale convergente a zero 

    Un sistema è stabile se tutti i suoi modi sono limitati: ho un
    autovalore nullo associato ad un modo naturale costante, e due
    autovalori a parte reale negativa, associati a due modi naturali
    sinusoidali contenuti da un esponenziale convergente a zero

    Per l'autovalore nullo serve verificarne la regolarità

    m.a. (molteplicità algebrica) = 1
    m.g. (molteplicità geometrica) = n-rank(A-lambda*eye(n))
    >> mg0=3-rank(A-0*eye(3))
    >> ans = 1

    Infatti 1<=mg(lambda)<=ma(lambda)<=n, per cui se la m.a. è unitaria, anche
    la m.g. è unitaria e pertanto l'autovalore sarà regolare



    Analizziamo anche la funzione di trasferimento (ricordando che i poli
    della fdt sono i coefficienti del polinomio caratteristico ottenibile
    con poly(A)):
    >> [num,den] = ss2tf(A,B,C,D)
    >> num =  0         0    1.1510    0.1774
    >> den = 1.0000    0.7390    0.9215         0
    che con un solo polo nullo e tutti gli altri negativi, mi da ulteriore
    conferma della stabilità



    Analizziamo l'evoluzione libera: nel Ciclo_Aperto.slx introduciamo un
    ingresso costante opportuno corrispondente ad uno stato di equilibrio.
    L'ingresso è nullo (ma inoltre, per sistemi lineri l'equilibrio è
    indifferente rispetto a quale ingresso costante si applichi). A tale
    ingresso il sistema si trova in equilibrio. Perturbando quest'ultimo
    con un du diverso da zero per un periodo di tempo T (es. 3 secondi)
    vediamo che l'uscita del sistema si assesta su un valore finito.
    Pertanto possiamo concludere che, a partire dalla perturbazione dello
    stato di equilibrio si innesca una evoluzione forzata. Spegnendo la
    perturbazione, il sistema si ritroverà in evoluzione libera con un
    ingresso nullo e delle condizioni iniziali scaturite dall'evoluzione
    forzata stessa. La risposta, assestandosi su un valore finito, mi
    permette di dire che il mio sistema è stabile

    Perciò possiamo concludere che il sistema è stabile
%}
% Plot degli autovalori
ALLFUNCS.Plot_eig('tc',eigA);
pause;

fprintf("\n Costante di tempo \n");
% Costante di tempo del sistema
tau=-1/min(real(eigA))
% Tempo finale di simulazione
tf=1.5*4*tau;
%{
    min(.) per prendere il modo più veloce di tutti, così la relativa
    costante di tempo sarà la più piccola di tutte

    1.5*4 = vedo un po' di più di 4 tau per eventuali effetti di deriva
%}

fprintf("\n MATRICE DI RAGGIUNGIBILITA \n")
% Calcolo matrice di raggiungibilità
R=ctrb(A,B)
% Verifico se il sistema è o NON è completamente raggiungibile
ALLFUNCS.VerificaProprieta(R, "raggiungibile");

fprintf("\n MATRICE DI OSSERVABILITA \n")
% Calcolo matrice di osservabilità
O=obsv(A,C)
% Verifico se il sistema è o NON è completamente osservabile
ALLFUNCS.VerificaProprieta(O, "osservabile");





%% Quesito 2 - Controllore con Retroazione dello stato con costante di tempo 5s
fprintf("\n\n\n Retroazione dello stato \n");

fprintf("\n Modello discretizzato \n");
% Parametri di discretizzazione
f=10; % campioni per ogni secondo
T=1/f; % secondi per ogni campione
% Discretizzo il modello
A_d=eye(3,3)+T*A
B_d=T*B
C_d=C

fprintf("\n Autovalori della matrice dinamica \n");
% Autovalori della matrice dinamica
eigA_d=eig(A_d)
alphai_d=min(real(eigA_d));
%{
    Il sistema, con due autovalori a modulo < 1 e uno solo a modulo = 1 è
    stabile
%}
% Plot degli autovalori
ALLFUNCS.Plot_eig('td',eigA_d);
pause;

fprintf("\n Costante di tempo \n");
% Costante di tempo del sistema
tau_d=floor(-1/min(log(abs(eigA_d)))) % passi
% Tempo finale di simulazione
tf_d=floor(1.5*4*tau_d); % passi totali di simulazione
k_d=1:tf_d; % vettore dei passi totali di simulazione
%{
    min(.) per prendere il modo più veloce di tutti, così la relativa
    costante di tempo sarà la più piccola di tutte

    1.5*4 = vedo un po' di più di 4 tau per eventuali effetti di deriva
%}

fprintf("\n MATRICE DI RAGGIUNGIBILITA \n")
% Calcolo matrice di raggiungibilità
R_d=ctrb(A_d,B_d)
% Verifico se il sistema è o NON è completamente raggiungibile
ALLFUNCS.VerificaProprieta(R_d, "raggiungibile dopo la discretizzazione");

fprintf("\n MATRICE DI OSSERVABILITA \n")
% Calcolo matrice di osservabilità
O_d=obsv(A_d,C_d)
% Verifico se il sistema è o NON è completamente osservabile
ALLFUNCS.VerificaProprieta(O_d, "osservabile dopo la discretizzazione");

% Inizializzo la costante di tempo richiesta
tau_rs=5; % secondi
tau_rs_d=tau_rs*f; % passi
% Tempo finale di simulazione
tf_rs_d=floor(1.5*4*tau_rs_d); % passi totali di simulazione
k_rs=1:tf_rs_d; % vettore dei passi totali di simulazione
% Condizioni iniziali del sistema
x0_rs=[0 0 deg*pi/180]';  % [alpha q teta]
% Autovalori desiderati
ldes_abs_rs=exp(-1/tau_rs_d);
ldes_rs=[ldes_abs_rs ldes_abs_rs-0.0001 ldes_abs_rs-0.0002]
%{
    Distanzio gli autovalori poichè il comando place incontra problemi
    numerici nel posizionare più autovalori nello stesso punto.
    Ingegneristicamente, cambiare di poco gli autovalori non cambia
    nulla, sia per come è definito tau che per quella che è
    l'approssimazione del sistema

    Assegnando una dinamica con autovalori tutti a modulo minore di uno si
    sta assegnando una dinamica asintoticamente stabile, pertanto ci si
    aspetta che lo stato converga a zero
%}
% Plot autovalori desiderati
ALLFUNCS.Plot_eig('td',eigA_d,ldes_rs');
pause;

% Calcolo il guadagno di retroazione
[Kr_rs]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_rs);

x_rs(:,1)=x0_rs;
for i=1:tf_rs_d
    u_rs(:,i)=Kr_rs*x_rs(:,i);
    y_rs(:,i)=C_d*x_rs(:,i)+odg_w*normrnd(0,1);
    if(i<tf_rs_d)
        x_rs(:,i+1)=A_d*x_rs(:,i)+B_d*u_rs(:,i);
    end
end

% Plot Retroazione dello stato
ALLFUNCS.Plot_rs(k_rs,x_rs,y_rs,true);
pause;





%% Quesito 3 - Retroazione dell'uscita con riferimento 0.05rad, condizioni iniziali nulle e passo di campionamento 100ms
fprintf("\n\n\n Retroazione dell'uscita \n");

% Uscita desiderata
yd=0.05;
% Condizioni iniziali
x0_ru=[0 0 0]'; % [alpha q teta]
% Autovalori desiderati
dec_ru=1.175; % velocità costante di tempo rispetto a tau del sistema
tau_ru_des=floor(tau_d/dec_ru); % tau desiderata corrispondente
ldes_abs_ru=exp(-1/tau_ru_des); % abs di ldes corrispondente
ldes_ru=[ldes_abs_ru ldes_abs_ru-0.0001 ldes_abs_ru-0.0002] % conoscendo eigA_d li voglio più veloci
%{
    Nel tempo discreto gli autovalori più veloci devono essere collocati
    più a sinistra di quelli veri, ma non troppo vicini lo 0. Non c'è una
    proporzionalità diretta come nel tempo continuo a causa del legame
    logaritmico che esiste tra tau e il modulo di lambda
%}
% Plot autovalori
ALLFUNCS.Plot_eig('td',eigA_d,ldes_ru');
pause;

% Calcolo del guadagno in retroazione
[Kr_ru]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_ru);

% Calcolo il guadagno di retroazione
[Ky]=ALLFUNCS.RetroazioneUscitaCicloAperto(A_d,B_d,C_d,Kr_ru);

x_ru(:,1)=x0_ru;
for i=1:tf_d
    u_ru(:,i)=(yd*Ky)+Kr_ru*x_ru(:,i);
    y_ru(:,i)=C_d*x_ru(:,i)+odg_w*normrnd(0,1);
    if(i<tf_d)
        x_ru(:,i+1)=A_d*x_ru(:,i)+B_d*u_ru(:,i);
    end 
end

% Plot Retroazione dell'uscita
ALLFUNCS.Plot_ru(k_d,x_ru,y_ru,true);
pause;





%% Quesito 4.1 - Osservatore deterministico
fprintf("\n\n\n Osservatore deterministico (Luenberger) \n");

% Condizioni iniziali del sistema
x0_od=[0 0 deg*pi/180]; % [alpha q teta]
% Condizioni iniziali osservatore richieste
x0hat_od=[0.05 0 0.01]'; % [alpha q teta]

% Autovalori desiderati per l'osservatore
dec_od=2; % velocità costante di tempo rispetto a tau del sistema
tau_od_des=floor(tau_d/dec_od); % tau desiderata corrispondente
ldes_abs_od=exp(-1/tau_od_des); % abs di ldes corrispondente
ldes_od=[ldes_abs_od ldes_abs_od-0.0001 ldes_abs_od-0.0002]
% Autovalori desiderati per il controllore
ldes_odr=[(alphai_d+ldes_od(1))/2 (alphai_d+ldes_od(1))/2-0.0001 (alphai_d+ldes_od(1))/2-0.0002]
%{
    Gli autovalori dell'osservatore devono essere i più veloci. Idealmente
    è un sensore, e pertanto lo voglio istantaneo, più veloce del sistema,
    ma anche più veloce della retroazione
%}
% Plot autovalori
ALLFUNCS.Plot_eig('td',eigA_d,ldes_odr',ldes_od');
pause;

% Osservatore
[Ko_od]=ALLFUNCS.OsservatoreDeterministico(A_d,C_d,ldes_od);

% Controllore
[Kr_od]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_odr);

x_od(:,1)=x0_od;
xhat_od(:,1)=x0hat_od;
for i=1:tf_d
    u_od(:,i)=Kr_od*xhat_od(:,i); % retroaziono la stima dello stato
    y_od(:,i)=C_d*x_od(:,i)+odg_w*normrnd(0,1);
    if(i<tf_d)
        x_od(:,i+1)=A_d*x_od(:,i)+B_d*u_od(:,i);
        xhat_od(:,i+1)=A_d*xhat_od(:,i)+B_d*u_od(:,i)+Ko_od*(y_od(:,i)-C_d*xhat_od(:,i));
    end
end

% Plot Osservatore deterministico
ALLFUNCS.Plot_od(k_d,x_od,xhat_od,y_od,true);
pause;





%% Quesito 4.2 - Osservatore stocastico
fprintf("\n\n\n Osservatore Stocastico (Kalman) \n");

% Autovalori desiderati per il controllore
ldes_k=ldes_odr;
% Plot autovalori
%ALLFUNCS.Plot_eig('td',eigA_d,ldes_k');
%pause;

% Controllore
[Kr_k]=ALLFUNCS.RetroazioneStato(A_d,B_d,C_d,ldes_k);

% Condizioni iniziali
x0_k=x0_od; % [alpha q teta]
% Inizializzazione filtro di Kalman
xhat_k(:,1)=[0.05 0 0.01]'; % stima a posteriori
xhats_k(:,1)=xhat_k(:,1); % stima a priori
% Variabili statistiche
Ps=[1e-1 0 0; 0 1e-3 0; 0 0 1e-1]; % covarianza errore di stima
R_w=[1e-1 0 0; 0 1e-3 0; 0 0 1e-1]; % covarianza rumore di processo
R_v=1e3; % covarianza rumore di misura

% Controllore-Osservatore
[x_k,xhat_k,y_k,K_k,Phist,Khist]=ALLFUNCS.OsservatoreStocastico(tf_d,Kr_k,xhat_k(:,1),xhats_k(:,1),x0_k,A_d,B_d,C_d,Ps,R_v,odg_w,R_w);

% Plot Kalman
ALLFUNCS.Plot_Kalman(k_d,x_k,xhat_k,y_k,Phist',Khist',true);