clc% 1. Extract time vectors (assuming modern Dataset or Structure with Time format)
% If you get an error here, check if your workspace contains a single structure named 'out'
time_vec1 = out.POSMAT.time; 

% 2. Extract the data arrays [Time x 3 Joints]
desired_pos1 = out.POSMAT.data;
joint_error1 = out.JPE.data;

% 3. Create a professional, clean multi-plot figure
figure('Color', 'w', 'Name', 'Matlab RoboticsToolbox CTC');

% --- Subplot 1: Desired Positions ---
subplot(3,1,1);
plot(time_vec1, desired_pos1, 'LineWidth', 1.5);
grid on;
title('Desired Joint Positions(Matlab CTC)');
ylabel('Position (rad)');
legend('Base', 'Shoulder', 'Wrist', 'Location', 'best');

% --- Subplot 2: Joint Position Errors ---
subplot(3,1,2);
plot(time_vec1, joint_error1, 'LineWidth', 1.5);
grid on;
title('Joint Position Tracking Errors(Matlab CTC)');
ylabel('Error (rad)');


