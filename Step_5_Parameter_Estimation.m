% Before you run This code check the dimension Of Beta in the workspace 
%% and Run the simulink File named dataextractForSecondExpValidation.slx
% Abdelrahman ELzemrany (Constrained Pos-Def Refactor - Fixed Handle)

k = 15;      % Dimension of Beta
time = 10;   % Experiment time
T = 0.001;   % Sampling period
l = time/T;  % Number of samples (10000)

%% Joint Data Extraction
q1 = pos.signals(1).values(1:l);  
q2 = pos.signals(2).values(1:l);   
q3 = pos.signals(3).values(1:l);  

qp1 = velocity.signals(1).values(1:l); 
qp2 = velocity.signals(2).values(1:l);  
qp3 = velocity.signals(3).values(1:l);  

qpp1 = accelration.signals(1).values(1:l);   
qpp2 = accelration.signals(2).values(1:l);  
qpp3 = accelration.signals(3).values(1:l); 

tau1 = torque.signals(1).values(1:l);   
tau2 = torque.signals(2).values(1:l); 
tau3 = torque.signals(3).values(1:l); 

tt = [tau1; tau2; tau3]; 

%% Rearranging The Observation Regressor Matrix 
fprintf('Assembling Regressor Matrix...');
Yb = zeros(3, k, l);
for i = 1:l
   Yb(:,:,i) = Y_b_handle(0,0,-9.81, q1(i),q2(i),q3(i), qp1(i),qp2(i),qp3(i), qpp1(i),qpp2(i),qpp3(i));
end

sum1 = zeros(l, k); sum2 = zeros(l, k); sum3 = zeros(l, k);
for ii = 1:k
    for ik = 1:l
        sum1(ik,ii) = Yb(1,ii,ik);
        sum2(ik,ii) = Yb(2,ii,ik);
        sum3(ik,ii) = Yb(3,ii,ik);
    end
end
Yc = [sum1; sum2; sum3]; 
fprintf(' Done.\n');

%% 1. Unconstrained Baseline (For comparison and Initial Guess)
theta_init = pinv(Yc'*Yc)*(Yc'*tt);

%% 2. Constrained Optimization (Enforcing Mass Matrix Positive Definiteness)
fprintf('Running Convex Constrained Optimization via fmincon...\n');

% Objective Function: Minimize sum of squared torque errors
obj_fun = @(theta) sum((Yc * theta - tt).^2);

% Optimization Settings
options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'Algorithm', 'interior-point', ...
    'MaxFunctionEvaluations', 30000, ...
    'MaxIterations', 1000);

% Define Joint Workspace Boundaries for Safety Scans (Adjust limits if needed)
q_lims = [-1.2, 1.2;       % Joint 1 Min/Max
          -2.4/2, 2.4/2;   % Joint 2 Min/Max
          -2.4/2, 2.4/2];  % Joint 3 Min/Max

% Execute fmincon with explicit function handle (@Y_b_handle)
[theta, fval, exitflag] = fmincon(obj_fun, theta_init, [], [], [], [], [], [], ...
    @(theta) mass_constraints(theta, @Y_b_handle, k, q_lims), options);

%% Evaluate Post-Constrained Metrics
e = tt - Yc*theta;     
tv = Yc*theta; 
tv1 = tv(1:l); 
tv2 = tv(l+1:2*l);
tv3 = tv(2*l+1:end);

%% Package for Simulink Validation Blocks
tau1t = timeseries(tau1); tau2t = timeseries(tau2); tau3t = timeseries(tau3);
tv1t = timeseries(tv1);   tv2t = timeseries(tv2);   tv3t = timeseries(tv3);

%% Calculate Estimation Accuracy Percentage (Fitness)
rmse1 = sqrt(mean((tau1 - tv1).^2));   denom1 = max(tau1) - min(tau1);   fit1 = (1 - (rmse1 / denom1)) * 100;
rmse2 = sqrt(mean((tau2 - tv2).^2));   denom2 = max(tau2) - min(tau2);   fit2 = (1 - (rmse2 / denom2)) * 100;
rmse3 = sqrt(mean((tau3 - tv3).^2));   denom3 = max(tau3) - min(tau3);   fit3 = (1 - (rmse3 / denom3)) * 100;
rmse_total = sqrt(mean((tt - tv).^2)); denom_total = max(tt) - min(tt); fit_overall = (1 - (rmse_total / denom_total)) * 100;

%% Print Final Verdict
fprintf('\n=================== CONSTRAINED MODEL ACCURACY ===================\n');
fprintf('Joint 1 (Base Yaw)   Accuracy: %6.2f %%\n', fit1);
fprintf('Joint 2 (Shoulder)   Accuracy: %6.2f %%\n', fit2);
fprintf('Joint 3 (Elbow)      Accuracy: %6.2f %%\n', fit3);
fprintf('-----------------------------------------------------\n');
fprintf('OVERALL CONSTRAINED ACCURACY: %6.2f %%\n', fit_overall);
fprintf('==================================================================\n');
if exitflag >= 1
    fprintf('SUCCESS: Parameter set is strictly positive-definite across your workspace limits!\n');
else
    fprintf('WARNING: Optimizer completed with issues. Check workspace grid constraints.\n');
end

%% ==================== MATCHED VISUALIZATION SECTION ====================
time_axis = (0:l-1) * T; % Time vector from 0 to 10 seconds

figure('Name', 'The Parameter estimation validation results', 'Color', 'w');
t_layout = tiledlayout(3, 1, 'TileSpacing', 'normal', 'Padding', 'compact'); 

actual_torques = {tau1, tau2, tau3};
estimated_torques = {tv1, tv2, tv3};
joint_titles = {'Joint 1 Dynamic Alignment', 'Joint 2 Dynamic Alignment', 'Joint 3 Dynamic Alignment'};

for j = 1:3
    nexttile(j);
    
    % Plot Style Matching: Solid Red line vs Dashed Blue line
    plot(time_axis, actual_torques{j}, 'r-', 'LineWidth', 1.2); hold on;
    plot(time_axis, estimated_torques{j}, 'b--', 'LineWidth', 1.0);
    
    % Axis Constraints & Grid Properties
    grid on;
    xlim([0, time]);
    ylabel('Torque (Nm)', 'FontSize', 10);
    title(joint_titles{j}, 'FontSize', 11, 'FontWeight', 'normal');
    
    % Only add X-label to the bottom plot to mimic publication style
    if j == 3
        xlabel('Time (s)', 'FontSize', 10);
    end
    
    % Match Legend Formatting from Reference Image
    if j == 1
        legend('Ideal Real Torque', 'Optimized Model', 'Location', 'northeast', 'FontSize', 9);
    end
end
%% =======================================================================

%% --- HELPER CONSTRAINT FUNCTION ---
function [c, ceq] = mass_constraints(theta, Y_b_handle, k, q_lims)
    ceq = []; % No equality constraints required
    c = [];   % Nonlinear inequality output vector (fmincon forces c <= 0)
    
    epsilon = 0.017; % The absolute minimum allowed eigenvalue limit for M(q)
    
    % Generate an auditing coordinate grid over key workspace limits
    q1_test = linspace(q_lims(1,1), q_lims(1,2), 5);
    q2_test = linspace(q_lims(2,1), q_lims(2,2), 5);
    q3_test = linspace(q_lims(3,1), q_lims(3,2), 5);
    
    % Loop through the grid to reconstruct M(q) dynamically using your handle
    for i = 1:5
        for j = 1:5
            for m = 1:5
                q = [q1_test(i); q2_test(j); q3_test(m)];
                
                % Numerically isolate the pure Mass Matrix columns via acceleration pulses
                M_curr = zeros(3,3);
                for col = 1:3
                    qpp_pulse = zeros(3,1);
                    qpp_pulse(col) = 1;
                    
                    % Pulse regressor with zero gravity/velocity, and 1 unit acceleration
                    Y_M_pulse = Y_b_handle(0,0,0, q(1),q(2),q(3), 0,0,0, qpp_pulse(1),qpp_pulse(2),qpp_pulse(3));
                    M_curr(:, col) = Y_M_pulse * theta;
                end
                
                % Calculate eigenvalues for this pose
                eg = eig(M_curr);
                
                % Constraint format: epsilon - min(eg) <= 0  ===> min(eg) >= epsilon
                c = [c; (epsilon - min(eg))]; %#ok<AGROW>
            end
        end
    end
end
