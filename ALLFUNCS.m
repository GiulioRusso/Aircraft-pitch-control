classdef ALLFUNCS
    methods(Static)
        %% Funzioni per il controllo
        
        % Funzione per verificare la proprietà di raggiungibilità o
        % osservabilità
        % - M = matrice da verificare
        % - proprieta = proprieta stampata da verificare
        function VerificaProprieta(M,proprieta)  
            [row, ~]=size(M);
            if rank(M)==row
                fprintf("\n Il sistema è completamente %s \n", proprieta)
            else
                fprintf("\n Il sistema NON è completamente %s \n", proprieta)
            end
        end
        
        % Funzione per la Retroazione dello stato passando gli autovalori
        % desiderati e restituendo il guadagno
        % - Kr = guadagno di retroazione dello stato
        % - A,B,C = matrici del sistema
        % - ldes = autovalori desiderati
        function [Kr]=RetroazioneStato(A,B,C,ldes)
            fprintf("\n GUADAGNO Kr \n");
            % Calcolo del guadagno in retroazione
            Kr=-place(A,B,ldes)
            fprintf("\n Verifica del guadagno Kr \n");
            % Verifica del calcolo del guadagno
            eig(A+B*Kr)
            % Verifica permanenza della completa osservabilità
            ALLFUNCS.VerificaProprieta(obsv(A+B*Kr,C),'osservabile dopo la retroazione dello stato');
        end
        
        % Funzione per la Retroazione dell'uscita passando gli autovalori
        % desiderati e restituendo i guadagni
        % - Ky = guadagno a ciclo aperto
        % - A,B,C = matrici del sistema
        % - Kr = guadagno di retroazione dello stato
        function [Ky]=RetroazioneUscitaCicloAperto(A,B,C,Kr)
            fprintf("\n GUADAGNO Ky \n");
            % Calcolo del guadagno a ciclo aperto
            Ky=inv(C*inv(eye(3)-A-B*Kr)*B)
        end
        
        % Funzione per il calcolo del guadagno dell'Osservatore
        % deterministico passando gli autovalori desiderati
        % - Ko = guadagno osservatore a ciclo chiuso
        % - A,C = matrici del sistema
        % - ldes = autovalori desiderati
        function [Ko]=OsservatoreDeterministico(A,C,ldes)
            fprintf("\n GUADAGNO Ko \n");
            % Calcolo del guadagno dell'osservatore
            Ko=place(A', C', ldes)'
            fprintf("\n Verifica del guadagno dell'osservatore \n");
            % Verifica del calcolo del guadagno
            eig(A-Ko*C)
        end
        
        % Funzione per la stima dello stato tramite Osservatore stocastico
        % e la validazione tramite retroazione del controllore-osservatore
        % - tf_d = durata (passi) della simulazione
        % - Kr = guadagno di retroazione dello stato
        % - xhat = condizione iniziale dell'osservatore
        % - xhats = condizione iniziale della stima a priori dello stato
        % - x0 = condizioni iniziali del sistema
        % - A,B,C = matrici del sistema
        % - Ps = condizione iniziale della stima a priori della covarianza
        % - R_v = covarianza del rumore di misura
        % - odg_w = ordine di grandezza del rumore sull'uscita
        % - R_w = covarianza del rumore di processo
        function [x,xhat,y,K,Phist,Khist]=OsservatoreStocastico(tf_d,Kr,xhat,xhats,x0,A,B,C,Ps,R_v,odg_w,R_w)
            % Inizializzo lo stato
            x(:,1) = x0;

            % Calcolo del Guadagno
            K=Ps*C'*inv(R_v+C*Ps*C');
            % Equazioni di aggiornamento di misura
            xhat(:,1);
            P=Ps-K*C*Ps;
            u(:,1)=Kr*xhat(:,1);
            % Equazioni di aggiornamento temporale
            xhats(:,2)=A*xhat(:,1)+B*u(:,1);
            Ps=A*P*A'+R_w;

            % Simulazione del sistema dinamico
            x(:,2)=A*x(:,1)+B*u(:,1);
            y(:,1)=C*x(:,1)+odg_w*normrnd(0,1);

            Phist(:,1) = svd(P);
            Khist(:,1) = min(svd(K));
            %{
                La prima iterazione è fatta fuori dal ciclo poichè la stima
                dello stato parte da una condizione iniziale nota, pertanto
                non è necessario stimarla già al primo passo. Quindi, la
                prima iterazione è fatta fuori, di modo da mantenere
                x_hat(:,1) pari a quello richiesto, per poi procedere con
                le iterazioni successive stimando lo stato come vuole
                l'algoritmo
            %}

            for i=2:tf_d
                % Acquisisco le misure
                y(:,i)=C*x(:,i)+odg_w*normrnd(0,1);
                % Calcolo del guadagno
                K=Ps*C'*inv(R_v+C*Ps*C');
                % Equazioni di aggiornamento di misura
                xhat(:,i)=xhats(:,i)+K*(y(:,i)-C*xhats(:,i));
                P=Ps-K*C*Ps;
                u(:,i)=Kr*xhat(:,i);
                % Equazioni di aggiornamento temporale
                if (i<tf_d)
                    xhats(:,i+1)=A*xhat(:,i)+B*u(:,i);
                    Ps=A*P*A'+R_w;
                end
                if (i<tf_d)
                    x(:,i+1)=A*x(:,i)+B*u(:,i);
                end
                Phist(:,i)=svd(P);
                Khist(:,i)=min(svd(K));
            end

            fprintf('\n GUADAGNO KALMAN \n');
            K
        end
        
        %% Funzioni di Plot
        
        % Funzione per la simulazione
        % - time = asse temporale della simulazione
        % - var = variabile del beccheggio
        function ShowSimulation(time,var)
            % Inizializzo i parametri di velocità della simulazione
            V=50;
            dt=0.05;
            step=4; % velocità di scorrimento dei campioni
            % Istanzio l'oggetto del velivolo
            a380=A380;
            % Imposto la mia view
            MyShow(a380);
            % Inizializzo il pith del velivolo
            a380.AddPitch(var(1));
            % Premere Invio per far partire la simulazione
            pause;
            % Inizio simulazione
            for t=1+step:step:length(time) % da 1 a length(time) con passo step
                % Il velivolo avanza in base alla velocità scelta
                a380.MoveForward(V*dt);
                % Aggiungo il delta del pitch
                a380.AddPitch(var(t)-var(t-step)); % differenza pari al passo
                % Mostro il valore del pitch del velivolo
                title(['\Theta = ',mat2str(var(t),1),' rad']);
                % Pausa tra un frame e l'altro
                pause(dt);
            end
        end
        
        % Plot degli autovalori del sistema
        % - tempdom = dominio temporale richiesto ('tc' disegna gli assi
        % cartesiani, 'td' disegna la circonferenza unitaria)
        % - eigval1 = autovalori del sistema
        % - eigval2 = autovalori desiderati per il controllo
        % - eigval3 = autovalori dell'osservatore
        function Plot_eig(tempdom,eigval1,eigval2,eigval3)
            if tempdom=='tc'
                f=figure;
                set(f,'Position',[400,400,640,600]);
                hold on;
                line([0,0], [-1.1 1.1], 'Color', '#F0F0F0', 'LineWidth', 2);
                line([-1.1 1.1], [0,0], 'Color', '#F0F0F0', 'LineWidth', 2);
                hold on
                plot(eigval1,'*','linewidth',1);
                hold on;
                text(real(eigval1(1)),imag(eigval1(1)),'\lambda_{1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                text(real(eigval1(2)),imag(eigval1(2)),'\lambda_{2}','VerticalAlignment','bottom','HorizontalAlignment','left');
                text(real(eigval1(3)),imag(eigval1(3)),'\lambda_{3}','VerticalAlignment','bottom','HorizontalAlignment','left');
                title(['Autovalori della matrice dinamica']);
                xlabel('\Re');
                ylabel('\Im');
                xlim([-1.1 1.1]);
                ylim([-1.1 1.1]);
                grid on;

                if ~exist('eigval2','var')
                     eigval2=0;
                else
                    hold on;
                    plot(eigval2,0,'*r','linewidth',1);
                    title(['Autovalori della matrice dinamica (\lambda) e desiderati (\lambda_{d})']);
                    hold on;
                    text(real(eigval2(1)),imag(eigval2(1)),'\lambda_{d1}','VerticalAlignment','bottom','HorizontalAlignment','right');
                    text(real(eigval2(2)),imag(eigval2(2)),'\lambda_{d2}','VerticalAlignment','top','HorizontalAlignment','left');
                    text(real(eigval2(3)),imag(eigval2(3)),'\lambda_{d3}','VerticalAlignment','top','HorizontalAlignment','right');
                end

                if ~exist('eigval3','var')
                     eigval3=0;
                else
                    hold on;
                    plot(eigval3,0,'*b','linewidth',1);
                    title(['Autovalori della matrice dinamica (\lambda), desiderati (\lambda_{d}) e dell osservatore (\lambda_{o})']);
                    hold on;
                    text(real(eigval3(1)),imag(eigval3(1)),'\lambda_{o1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                    text(real(eigval3(2)),imag(eigval3(2)),'\lambda_{o2}','VerticalAlignment','top','HorizontalAlignment','left');
                    text(real(eigval3(3)),imag(eigval3(3)),'\lambda_{o3}','VerticalAlignment','bottom','HorizontalAlignment','right');
                end
            elseif tempdom=='td'
                f=figure;
                set(f,'Position',[400,400,640,600]);
                hold on;
                viscircles([0 0], 1,'LineStyle','-','Color','#F0F0F0','Linewidth',2);
                hold on;
                line([0,0], [-1.1 1.1], 'Color', '#F0F0F0', 'LineWidth', 2);
                line([-1.1 1.1], [0,0], 'Color', '#F0F0F0', 'LineWidth', 2);
                hold on;
                plot(eigval1,'*','linewidth',1);
                hold on;
                text(real(eigval1(1)),imag(eigval1(1)),'\lambda_{1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                text(real(eigval1(2)),imag(eigval1(2)),'\lambda_{2}','VerticalAlignment','bottom','HorizontalAlignment','left');
                text(real(eigval1(3)),imag(eigval1(3)),'\lambda_{3}','VerticalAlignment','bottom','HorizontalAlignment','left');
                title(['Autovalori della matrice dinamica']);
                xlabel('\Re');
                ylabel('\Im');
                xlim([-1.1 1.1]);
                ylim([-1.1 1.1]);
                grid on;
                
                if ~exist('eigval2','var')
                     eigval2=0;
                else
                    hold on;
                    plot(eigval2,0,'*r','linewidth',1);
                    title(['Autovalori della matrice dinamica (\lambda) e desiderati (\lambda_{d})']);
                    hold on;
                    text(real(eigval2(1)),imag(eigval2(1)),'\lambda_{d1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                    text(real(eigval2(2)),imag(eigval2(2)),'\lambda_{d2}','VerticalAlignment','top','HorizontalAlignment','left');
                    text(real(eigval2(3)),imag(eigval2(3)),'\lambda_{d3}','VerticalAlignment','bottom','HorizontalAlignment','right');
                end
                
                if ~exist('eigval3','var')
                     eigval3=0;
                else
                    hold on;
                    plot(eigval3,0,'*b','linewidth',1);
                    title(['Autovalori della matrice dinamica (\lambda), desiderati (\lambda_{d}) e dell osservatore (\lambda_{o})']);
                    hold on;
                    text(real(eigval3(1)),imag(eigval3(1)),'\lambda_{o1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                    text(real(eigval3(2)),imag(eigval3(2)),'\lambda_{o2}','VerticalAlignment','top','HorizontalAlignment','left');
                    text(real(eigval3(3)),imag(eigval3(3)),'\lambda_{o3}','VerticalAlignment','bottom','HorizontalAlignment','right');
                end
            end  
        end
        
        % Plot retroazione dello stato
        % - k = asse temporale
        % - x = stato
        % - y = uscita
        % - flagsim = flag per la simulazione
        function Plot_rs(k,x,y,flagsim)
            f=figure('Name','Retroazione dello Stato');
            set(f,'Position',[100,100,1600,2560]);
            subplot(2,2,1);
            plot(k,x,'linewidth',2,'markersize',3);
            pbaspect([1 1 1]);
            title('Stato');
            legend('Angolo di attacco','Pitch Rate','Pitch');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;
            subplot(2,2,3);
            plot(k,y,'linewidth',2,'markersize',3);
            pbaspect([1 1 1]);
            title('Uscita');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                subplot(2,2,[2,4]);
                ALLFUNCS.ShowSimulation(k,y);
            end
        end
        
        % Plot retroazione dell'uscita
        % - k = asse temporale
        % - x = stato
        % - y = uscita
        % - flagsim = flag per la simulazione
        function Plot_ru(k,x,y,flagsim)
            f=figure('Name','Retroazione dell Uscita');
            set(f,'Position',[100,100,1600,2560]);
            subplot(2,2,1);
            plot(k,x,'linewidth',2,'markersize',3);
            title('Stato');
            legend({'Angolo di attacco','Pitch Rate','Pitch'},'Location','southeast');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;
            subplot(2,2,3);
            plot(k,y,'linewidth',2,'markersize',3);
            title('Uscita');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                subplot(2,2,[2,4]);
                ALLFUNCS.ShowSimulation(k,y)
            end            
        end

        % Plot controllore-osservatore deterministico
        % - k = asse temporale
        % - x = stato
        % - xhat = stato stimato
        % - y = uscita
        % - flagsim = flag per la simulazione
        function Plot_od(k,x,xhat,y,flagsim)
            f=figure('Name','Osservatore Deterministico (Luenberger)');
            set(f,'Position',[0,0,720,1250]);
            subplot(3,3,1);
            plot(k,x(1,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(1,:),'--','linewidth',2,'markersize',3);
            title('Angolo di attacco');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,2);
            plot(k,x(2,:),'linewidth',2,'markersize',3);
            hold on
            plot(k,xhat(2,:),'--','linewidth',2,'markersize',3);
            title('Pitch Rate');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,3);
            plot(k,x(3,:),'linewidth',2,'markersize',3);
            hold on
            plot(k,xhat(3,:),'--','linewidth',2,'markersize',3);
            title('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,4);
            plot(k,x(1,:)-xhat(1,:),'linewidth',2,'markersize',3);
            hold on
            title('Errore Angolo di attacco');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,5);
            plot(k,x(2,:)-xhat(2,:),'linewidth',2,'markersize',3);
            hold on
            title('Errore Pitch Rate');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,6);
            plot(k,x(3,:)-xhat(3,:),'linewidth',2,'markersize',3);
            hold on
            title('Errore Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,[7,9]);
            plot(k,y,'linewidth',2,'markersize',3);
            title('Uscita');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                h=figure('Name','Simulazione Osservatore Deterministico (Luenberger)');
                set(h,'Position',[720,0,720,1250]);
                subplot(2,1,1);
                title(['Simulazione con lo stato stimato (tratteggiato)']);
                ALLFUNCS.ShowSimulation(k,xhat(3,:));
                subplot(2,1,2);
                title(['Simulazione con lo stato vero (continuo)']);
                ALLFUNCS.ShowSimulation(k,x(3,:));
            end 
        end
        
        % Plot controllore-osservatore stocastico
        % - k = asse temporale
        % - x = stato
        % - xhat = stato stimato
        % - y = uscita
        % - Phist e Khist = svd di P e K
        % - flagsim = flag per la simulazione
        function Plot_Kalman(k,x,xhat,y,Phist,Khist,flagsim)
            f=figure('Name','Osservatore Stocastico (Kalman)');
            set(f,'Position',[0,0,720,1250]);
            subplot(3,3,1);
            plot(k,x(1,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(1,:),'--','linewidth',2,'markersize',3);
            title('Angolo di attacco');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,2);
            plot(k,x(2,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(2,:),'--','linewidth',2,'markersize',3);
            title('Pitch Rate');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,3);
            plot(k,x(3,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(3,:),'--','linewidth',2,'markersize',3);
            title('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,4);
            plot(k,x(1,:)-xhat(1,:),'linewidth',2,'markersize',3);
            hold on;
            title('Errore Angolo di attacco');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,5);
            plot(k,x(2,:)-xhat(2,:),'linewidth',2,'markersize',3);
            hold on;
            title('Errore Pitch Rate');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,6);
            plot(k,x(3,:)-xhat(3,:),'linewidth',2,'markersize',3);
            hold on;
            title('Errore Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            subplot(3,3,7);
            plot(k,Phist,'linewidth',2,'markersize',3);
            title('svd P');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,8);
            plot(k,Khist,'linewidth',2,'markersize',3);
            title('svd K');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;
            
            subplot(3,3,9);
            plot(k,y,'linewidth',2,'markersize',3);
            title('Uscita');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                h=figure('Name','Simulazione Osservatore Stocastico (Kalman)');
                set(h,'Position',[720,0,720,1250]);
                subplot(2,1,1);
                title(['Simulazione con lo stato stimato (tratteggiato)']);
                ALLFUNCS.ShowSimulation(k,xhat(3,:));
                subplot(2,1,2);
                title(['Simulazione con lo stato vero (continuo)']);
                ALLFUNCS.ShowSimulation(k,x(3,:));
            end
        end
        
    end
end