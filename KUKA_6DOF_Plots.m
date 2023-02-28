function KUKA_6DOF_Plots(t1, p_comp, x1, N, t, y1, t2, y2, z1, p_comp_dot, y1_dot, y2_dot, q_out, q_out_dot)

    figure(2);
    subplot(3,1,1);
    plot(t1, p_comp(1,:), 'b-');
    grid;
    hold on;
    plot(t1, x1*ones(1,N), 'r-.');
    xlabel("time [sec]");
    ylabel("p_E_x(t) [m]");
    legend('Computed', 'Desired', 'Location', 'Southwest');
    hold off;

    subplot(3,1,2);
    plot(t1, p_comp(2,:), 'b-');
    grid;
    hold on;
    plot(t, y1, 'r-.');
    plot(t2, y2, 'r-.');
    xlabel("time [sec]");
    ylabel("p_E_y(t) [m]");
    legend('Computed', 'Desired', 'Location', 'Northwest');
    hold off;

    subplot(3,1,3);
    plot(t1, p_comp(3,:), 'b-');
    grid;
    hold on;
    plot(t1, z1*ones(1,N), 'r-.');
    xlabel("time [sec]");
    ylabel("p_E_z(t) [m]");
    legend('Computed', 'Desired', 'Location', 'Northwest');
    hold off;

    figure(3);
    subplot(3,1,1);
    plot(t1, p_comp_dot(1,:), 'b-');
    grid;
    hold on;
    plot(t1, zeros(1,N), 'r-.');
    xlabel("time [sec]");
    ylabel("v_E_x(t)  [m/sec]");
    legend('Computed', 'Desired', 'Location', 'Southwest');
    hold off;

    subplot(3,1,2);
    plot(t1, p_comp_dot(2,:), 'b-');
    grid;
    hold on;
    plot(t, y1_dot, 'r-.');
    plot(t2, y2_dot, 'r-.');
    xlabel("time [sec]");
    ylabel("v_E_y(t) [m/sec]");
    legend('Computed', 'Desired', 'Location', 'Southwest');
    hold off;

    subplot(3,1,3);
    plot(t1, p_comp_dot(3,:), 'b-');
    grid;
    hold on;
    plot(t1, zeros(1,N), 'r-.');
    xlabel("time [sec]");
    ylabel("v_E_z(t) [m/sec]");
    legend('Computed', 'Desired', 'Location', 'Southwest');
    hold off;


    figure(4);
    subplot(3,1,1);
    plot(t1, q_out(1,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("q_1(t) [rad]");

    subplot(3,1,2);
    plot(t1, q_out(2,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("q_2(t) [rad]");

    subplot(3,1,3);
    plot(t1, q_out(3,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("q_3(t) [rad]");

    figure(5);
    subplot(3,1,1);
    plot(t1, q_out_dot(1,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("qvel_1(t) [rad/sec]");

    subplot(3,1,2);
    plot(t1, q_out_dot(2,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("qvel_2(t) [rad/sec]");

    subplot(3,1,3);
    plot(t1, q_out_dot(3,:), 'r-');
    grid;
    xlabel("time [sec]");
    ylabel("qvel_3(t) [rad/sec]");
end