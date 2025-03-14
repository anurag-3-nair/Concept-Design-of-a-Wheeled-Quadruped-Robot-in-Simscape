function plot_angles_of_joints(save,time, struct)    
    figure;
    subplot(3, 1, 1);
    plot(time, struct.RL_Si.signals.values);
    hold on;
    plot(time, struct.RR_Si.signals.values);
    plot(time, struct.FL_Si.signals.values);
    plot(time, struct.FR_Si.signals.values);
    xlabel('Time');
    ylabel('Angles');
    title('Sideways Joints');
    hold off

    subplot(3, 1, 2);
    plot(time, struct.RL_Sh.signals.values);
    hold on
    plot(time, struct.RR_Sh.signals.values);
    plot(time, struct.FL_Sh.signals.values);
    plot(time, struct.FR_Sh.signals.values);
    xlabel('Time');
    ylabel('Angles');
    title('Shoulder Joints');
    hold off

    subplot(3, 1, 3);
    plot(time, struct.RL_Kn.signals.values);
    hold on
    plot(time, struct.RR_Kn.signals.values);
    plot(time, struct.FL_Kn.signals.values);
    plot(time, struct.FR_Kn.signals.values);
    xlabel('Time');
    ylabel('Angles');
    title('Knee Joints');
    hold off

    if save
        timestamp = datetime('now', 'Format', 'dd_MM-HH_mm');
        filename = ['Workflows\complete_robot\plots_for_debug\several_signals\temp_plot-' char(timestamp) '.fig'];
        saveas(gcf, filename);
    end
end