classdef ALLFUNCS
    methods(Static)
        %% Control functions
        
        % Check the reachability or observability property
        % - M = matrix to check
        % - proprieta = name of the property, printed in the message
        function VerificaProprieta(M,proprieta)  
            [row, ~]=size(M);
            if rank(M)==row
                fprintf("\n The system is completely %s \n", proprieta)
            else
                fprintf("\n The system is NOT completely %s \n", proprieta)
            end
        end
        
        % State feedback: takes the desired eigenvalues and returns the gain
        % - Kr = state feedback gain (u = Kr*x)
        % - A,B,C = system matrices
        % - ldes = desired eigenvalues
        function [Kr]=RetroazioneStato(A,B,C,ldes)
            fprintf("\n GAIN Kr \n");
            % Compute the feedback gain
            Kr=-place(A,B,ldes)
            fprintf("\n Check of the gain Kr \n");
            % Check the computed gain
            eig(A+B*Kr)
            % Check that complete observability is preserved
            ALLFUNCS.VerificaProprieta(obsv(A+B*Kr,C),'observable after state feedback');
        end
        
        % Output tracking: feedforward gain that gives unit DC gain from the
        % reference to the output of the closed loop (A+B*Kr, B, C)
        % - Ky = open-loop (feedforward) gain
        % - A,B,C = system matrices
        % - Kr = state feedback gain
        function [Ky]=RetroazioneUscitaCicloAperto(A,B,C,Kr)
            fprintf("\n GAIN Ky \n");
            % Compute the open-loop (feedforward) gain
            Ky=inv(C*inv(eye(3)-A-B*Kr)*B)
        end
        
        % Deterministic (Luenberger) observer gain from the desired
        % eigenvalues
        % - Ko = observer gain
        % - A,C = system matrices
        % - ldes = desired eigenvalues
        function [Ko]=OsservatoreDeterministico(A,C,ldes)
            fprintf("\n GAIN Ko \n");
            % Compute the observer gain
            Ko=place(A', C', ldes)'
            fprintf("\n Check of the observer gain \n");
            % Check the computed gain
            eig(A-Ko*C)
        end
        
        % State estimation with the stochastic (Kalman) observer, validated
        % in a controller-observer loop
        % - tf_d = simulation length (steps)
        % - Kr = state feedback gain
        % - xhat = initial condition of the observer
        % - xhats = initial a priori state estimate
        % - x0 = initial conditions of the system
        % - A,B,C = system matrices
        % - Ps = initial a priori error covariance
        % - R_v = measurement noise covariance
        % - odg_w = order of magnitude of the noise on the output
        % - R_w = process noise covariance
        function [x,xhat,y,K,Phist,Khist]=OsservatoreStocastico(tf_d,Kr,xhat,xhats,x0,A,B,C,Ps,R_v,odg_w,R_w)
            % Initialise the state
            x(:,1) = x0;

            % Compute the gain
            K=Ps*C'*inv(R_v+C*Ps*C');
            % Measurement update equations
            xhat(:,1);
            P=Ps-K*C*Ps;
            u(:,1)=Kr*xhat(:,1);
            % Time update equations
            xhats(:,2)=A*xhat(:,1)+B*u(:,1);
            Ps=A*P*A'+R_w;

            % Simulate the dynamic system
            x(:,2)=A*x(:,1)+B*u(:,1);
            y(:,1)=C*x(:,1)+odg_w*normrnd(0,1);

            Phist(:,1) = svd(P);
            Khist(:,1) = min(svd(K));
            %{
                The first iteration is done outside the loop because the
                state estimate starts from a known initial condition, so
                there is no need to estimate it at the first step. This
                keeps x_hat(:,1) equal to the required value; the following
                iterations then estimate the state as the algorithm
                prescribes
            %}

            for i=2:tf_d
                % Acquire the measurements
                y(:,i)=C*x(:,i)+odg_w*normrnd(0,1);
                % Compute the gain
                K=Ps*C'*inv(R_v+C*Ps*C');
                % Measurement update equations
                xhat(:,i)=xhats(:,i)+K*(y(:,i)-C*xhats(:,i));
                P=Ps-K*C*Ps;
                u(:,i)=Kr*xhat(:,i);
                % Time update equations
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

            fprintf('\n KALMAN GAIN \n');
            K
        end
        
        %% Plot functions
        
        % 3D animation of the aircraft
        % - time = simulation time axis
        % - var = pitch angle
        function ShowSimulation(time,var)
            % Animation speed parameters
            V=50;
            dt=0.05;
            step=4; % samples skipped per frame
            % Create the aircraft object
            a380=A380;
            % Set the view
            MyShow(a380);
            % Initialise the aircraft pitch
            a380.AddPitch(var(1));
            % Press Enter to start the animation
            pause;
            % Start the animation
            for t=1+step:step:length(time) % from 1 to length(time) with increment step
                % The aircraft moves forward at the chosen speed
                a380.MoveForward(V*dt);
                % Add the pitch increment
                a380.AddPitch(var(t)-var(t-step)); % difference over one increment
                % Show the aircraft pitch value
                title(['\Theta = ',mat2str(var(t),1),' rad']);
                % Pause between frames
                pause(dt);
            end
        end
        
        % Plot the system eigenvalues
        % - tempdom = time domain ('tc' continuous time draws the Cartesian
        % axes, 'td' discrete time draws the unit circle)
        % - eigval1 = system eigenvalues
        % - eigval2 = desired eigenvalues for the controller
        % - eigval3 = observer eigenvalues
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
                title(['Eigenvalues of the state matrix']);
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
                    title(['Eigenvalues of the state matrix (\lambda) and desired (\lambda_{d})']);
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
                    title(['Eigenvalues of the state matrix (\lambda), desired (\lambda_{d}) and observer (\lambda_{o})']);
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
                title(['Eigenvalues of the state matrix']);
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
                    title(['Eigenvalues of the state matrix (\lambda) and desired (\lambda_{d})']);
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
                    title(['Eigenvalues of the state matrix (\lambda), desired (\lambda_{d}) and observer (\lambda_{o})']);
                    hold on;
                    text(real(eigval3(1)),imag(eigval3(1)),'\lambda_{o1}','VerticalAlignment','bottom','HorizontalAlignment','left');
                    text(real(eigval3(2)),imag(eigval3(2)),'\lambda_{o2}','VerticalAlignment','top','HorizontalAlignment','left');
                    text(real(eigval3(3)),imag(eigval3(3)),'\lambda_{o3}','VerticalAlignment','bottom','HorizontalAlignment','right');
                end
            end  
        end
        
        % Plot state feedback
        % - k = time axis
        % - x = state
        % - y = output
        % - flagsim = show the 3D animation
        function Plot_rs(k,x,y,flagsim)
            f=figure('Name','State Feedback');
            set(f,'Position',[100,100,1600,2560]);
            subplot(2,2,1);
            plot(k,x,'linewidth',2,'markersize',3);
            pbaspect([1 1 1]);
            title('State');
            legend('Angle of attack','Pitch Rate','Pitch');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;
            subplot(2,2,3);
            plot(k,y,'linewidth',2,'markersize',3);
            pbaspect([1 1 1]);
            title('Output');
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
        
        % Plot output feedback
        % - k = time axis
        % - x = state
        % - y = output
        % - flagsim = show the 3D animation
        function Plot_ru(k,x,y,flagsim)
            f=figure('Name','Output Feedback');
            set(f,'Position',[100,100,1600,2560]);
            subplot(2,2,1);
            plot(k,x,'linewidth',2,'markersize',3);
            title('State');
            legend({'Angle of attack','Pitch Rate','Pitch'},'Location','southeast');
            xlabel('time [-]');
            xlim([0 k(length(k))]);
            grid on;
            subplot(2,2,3);
            plot(k,y,'linewidth',2,'markersize',3);
            title('Output');
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

        % Plot deterministic controller-observer
        % - k = time axis
        % - x = state
        % - xhat = estimated state
        % - y = output
        % - flagsim = show the 3D animation
        function Plot_od(k,x,xhat,y,flagsim)
            f=figure('Name','Deterministic Observer (Luenberger)');
            set(f,'Position',[0,0,720,1250]);
            subplot(3,3,1);
            plot(k,x(1,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(1,:),'--','linewidth',2,'markersize',3);
            title('Angle of attack');
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
            title('Angle of attack error');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,5);
            plot(k,x(2,:)-xhat(2,:),'linewidth',2,'markersize',3);
            hold on
            title('Pitch rate error');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,6);
            plot(k,x(3,:)-xhat(3,:),'linewidth',2,'markersize',3);
            hold on
            title('Pitch error');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,[7,9]);
            plot(k,y,'linewidth',2,'markersize',3);
            title('Output');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                h=figure('Name','Animation Deterministic Observer (Luenberger)');
                set(h,'Position',[720,0,720,1250]);
                subplot(2,1,1);
                title(['Animation with the estimated state (dashed)']);
                ALLFUNCS.ShowSimulation(k,xhat(3,:));
                subplot(2,1,2);
                title(['Animation with the true state (solid)']);
                ALLFUNCS.ShowSimulation(k,x(3,:));
            end 
        end
        
        % Plot stochastic controller-observer
        % - k = time axis
        % - x = state
        % - xhat = estimated state
        % - y = output
        % - Phist and Khist = svd of P and K
        % - flagsim = show the 3D animation
        function Plot_Kalman(k,x,xhat,y,Phist,Khist,flagsim)
            f=figure('Name','Stochastic Observer (Kalman)');
            set(f,'Position',[0,0,720,1250]);
            subplot(3,3,1);
            plot(k,x(1,:),'linewidth',2,'markersize',3);
            hold on;
            plot(k,xhat(1,:),'--','linewidth',2,'markersize',3);
            title('Angle of attack');
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
            title('Angle of attack error');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,5);
            plot(k,x(2,:)-xhat(2,:),'linewidth',2,'markersize',3);
            hold on;
            title('Pitch rate error');
            xlabel('time [-]');
            ylabel('angle rate [-]');
            xlim([0 k(length(k))]);
            grid on;

            subplot(3,3,6);
            plot(k,x(3,:)-xhat(3,:),'linewidth',2,'markersize',3);
            hold on;
            title('Pitch error');
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
            title('Output');
            legend('Pitch');
            xlabel('time [-]');
            ylabel('angle [rad]');
            xlim([0 k(length(k))]);
            grid on;
            
            if flagsim==true
                h=figure('Name','Animation Stochastic Observer (Kalman)');
                set(h,'Position',[720,0,720,1250]);
                subplot(2,1,1);
                title(['Animation with the estimated state (dashed)']);
                ALLFUNCS.ShowSimulation(k,xhat(3,:));
                subplot(2,1,2);
                title(['Animation with the true state (solid)']);
                ALLFUNCS.ShowSimulation(k,x(3,:));
            end
        end
        
    end
end