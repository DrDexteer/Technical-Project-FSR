%% PLOT NOMINAL SIMULATION RESULTS

close all;

% Open all figures as docked tabs in the MATLAB figure window
set(0,'DefaultFigureWindowStyle','docked');

controller_name = "INDI";   % "Geometric", "Hierarchical", "INDI"

fig_folder = "figures_nominal";
if ~exist(fig_folder, "dir")
    mkdir(fig_folder);
end

%% Extract common data from SimulationOutput object

t_sim = out.err_p_log.Time;

err_p = ts_to_matrix(out.err_p_log);     % [m]
tau   = ts_to_matrix(out.tau_log);       % [N m]
uT    = ts_to_matrix(out.uT_log);        % [N]

% Estimated external wrench
% The expected structure is:
% hat_w_e = [hat_f_ext; hat_tau_ext] in R^6
hat_w_e_ts = get_logged_signal(out, ["hat_w_e", "hat_w_e_log"]);
t_wrench   = hat_w_e_ts.Time;

hat_w_e = ts_to_matrix(hat_w_e_ts);

if size(hat_w_e,2) ~= 6
    error("hat_w_e must have 6 columns: [hat_f_ext_x hat_f_ext_y hat_f_ext_z hat_tau_ext_x hat_tau_ext_y hat_tau_ext_z].");
end

hat_f_ext   = hat_w_e(:,1:3);      % estimated external force [N]
hat_tau_ext = hat_w_e(:,4:6);      % estimated external torque [N m]

%% Select attitude logs depending on controller

if controller_name == "Hierarchical"

    % Hierarchical controller uses eta and dot eta errors
    err_att  = ts_to_matrix(out.err_eta_log);       % [rad]
    err_rate = ts_to_matrix(out.err_dot_eta_log);   % [rad/s]

    att_title  = "Eta Error";
    rate_title = "Dot Eta Error";

    att_labels = {'$e_\phi$ [rad]', ...
                  '$e_\theta$ [rad]', ...
                  '$e_\psi$ [rad]'};

    rate_labels = {'$e_{\dot{\phi}}$ [rad/s]', ...
                   '$e_{\dot{\theta}}$ [rad/s]', ...
                   '$e_{\dot{\psi}}$ [rad/s]'};

    rate_file_name = "_dot_eta_error";

else

    % Geometric and INDI controllers use SO(3) attitude error
    err_att  = ts_to_matrix(out.err_R_log);     % [-]
    err_rate = ts_to_matrix(out.err_W_log);     % [rad/s]

    att_title  = "Attitude Error";
    rate_title = "Angular Velocity Error";

    att_labels = {'$e_{R,1}$ [-]', ...
                  '$e_{R,2}$ [-]', ...
                  '$e_{R,3}$ [-]'};

    rate_labels = {'$e_{\omega,1}$ [rad/s]', ...
                   '$e_{\omega,2}$ [rad/s]', ...
                   '$e_{\omega,3}$ [rad/s]'};

    rate_file_name = "_angular_velocity_error";

end

%% Position error components

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_sim, err_p(:,1), 'LineWidth', 1.2);
grid on;
ylabel('$e_x$ [m]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Position Error");

nexttile;
plot(t_sim, err_p(:,2), 'LineWidth', 1.2);
grid on;
ylabel('$e_y$ [m]', 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_sim, err_p(:,3), 'LineWidth', 1.2);
grid on;
ylabel('$e_z$ [m]', 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_position_error.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_position_error.fig"));

%% Position error norm

err_p_norm = vecnorm(err_p, 2, 2);    % [m]

figure('Color','w');
plot(t_sim, err_p_norm, 'LineWidth', 1.4);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('$\|e_p\|$ [m]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Position Error Norm");

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_position_error_norm.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_position_error_norm.fig"));

%% Attitude / eta error

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_sim, err_att(:,1), 'LineWidth', 1.2);
grid on;
ylabel(att_labels{1}, 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - " + att_title);

nexttile;
plot(t_sim, err_att(:,2), 'LineWidth', 1.2);
grid on;
ylabel(att_labels{2}, 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_sim, err_att(:,3), 'LineWidth', 1.2);
grid on;
ylabel(att_labels{3}, 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_attitude_error.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_attitude_error.fig"));

%% Dot eta error / angular velocity error

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_sim, err_rate(:,1), 'LineWidth', 1.2);
grid on;
ylabel(rate_labels{1}, 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - " + rate_title);

nexttile;
plot(t_sim, err_rate(:,2), 'LineWidth', 1.2);
grid on;
ylabel(rate_labels{2}, 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_sim, err_rate(:,3), 'LineWidth', 1.2);
grid on;
ylabel(rate_labels{3}, 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + rate_file_name + ".png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + rate_file_name + ".fig"));

%% Control torque

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_sim, tau(:,1), 'LineWidth', 1.2);
grid on;
ylabel('$\tau_x$ [N m]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Control Torque");

nexttile;
plot(t_sim, tau(:,2), 'LineWidth', 1.2);
grid on;
ylabel('$\tau_y$ [N m]', 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_sim, tau(:,3), 'LineWidth', 1.2);
grid on;
ylabel('$\tau_z$ [N m]', 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_torque.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_torque.fig"));

%% Total thrust

figure('Color','w');
plot(t_sim, uT, 'LineWidth', 1.4);
grid on;
xlabel('time [s]', 'FontSize', 12);
ylabel('$u_T$ [N]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Total Thrust");

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_thrust.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_thrust.fig"));

%% Estimated external force

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_wrench, hat_f_ext(:,1), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{f}_{e,x}$ [N]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Estimated External Force");

nexttile;
plot(t_wrench, hat_f_ext(:,2), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{f}_{e,y}$ [N]', 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_wrench, hat_f_ext(:,3), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{f}_{e,z}$ [N]', 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_estimated_external_force.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_estimated_external_force.fig"));

%% Estimated external torque

figure('Color','w');
tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_wrench, hat_tau_ext(:,1), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{\tau}_{e,x}$ [N m]', 'Interpreter','latex', 'FontSize', 12);
title(controller_name + " - Estimated External Torque");

nexttile;
plot(t_wrench, hat_tau_ext(:,2), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{\tau}_{e,y}$ [N m]', 'Interpreter','latex', 'FontSize', 12);

nexttile;
plot(t_wrench, hat_tau_ext(:,3), 'LineWidth', 1.2);
grid on;
ylabel('$\hat{\tau}_{e,z}$ [N m]', 'Interpreter','latex', 'FontSize', 12);
xlabel('time [s]', 'FontSize', 12);

exportgraphics(gcf, fullfile(fig_folder, controller_name + "_estimated_external_torque.png"), 'Resolution', 300);
savefig(gcf, fullfile(fig_folder, controller_name + "_estimated_external_torque.fig"));

%% Save numerical results

max_err_p = max(err_p_norm);
rms_err_p = rms(err_p_norm);

err_att_norm = vecnorm(err_att, 2, 2);
max_err_att = max(err_att_norm);
rms_err_att = rms(err_att_norm);

err_rate_norm = vecnorm(err_rate, 2, 2);
max_err_rate = max(err_rate_norm);
rms_err_rate = rms(err_rate_norm);

tau_norm = vecnorm(tau, 2, 2);
max_tau = max(tau_norm);
rms_tau = rms(tau_norm);

J_tau = sum(vecnorm(diff(tau), 2, 2));

metrics = table(max_err_p, rms_err_p, ...
                max_err_att, rms_err_att, ...
                max_err_rate, rms_err_rate, ...
                max_tau, rms_tau, J_tau);

disp(metrics);

save(fullfile(fig_folder, controller_name + "_nominal_results.mat"), ...
    "t_sim", ...
    "t_wrench", ...
    "err_p", "err_p_norm", ...
    "err_att", "err_att_norm", ...
    "err_rate", "err_rate_norm", ...
    "tau", "uT", ...
    "hat_w_e", "hat_f_ext", "hat_tau_ext", ...
    "metrics");

%% Helper functions

function X = ts_to_matrix(ts)

    X = squeeze(ts.Data);
    t_len = length(ts.Time);

    if isvector(X)
        X = X(:);
    else
        if size(X,1) ~= t_len && size(X,2) == t_len
            X = X';
        end
    end

end

function ts = get_logged_signal(simOut, candidate_names)

    available_signals = string(simOut.who);

    for k = 1:numel(candidate_names)
        name_k = candidate_names(k);

        if any(available_signals == name_k)
            ts = simOut.get(char(name_k));
            return;
        end
    end

    error("None of the requested logged signals was found: %s", strjoin(candidate_names, ", "));

end