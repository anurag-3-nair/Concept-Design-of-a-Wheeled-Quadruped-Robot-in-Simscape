function plot_temp_over_t(save, struct)
    time = struct.time;
    data = struct.signals.values;
    figure;
    plot(time, data);
    xlabel('Time');
    ylabel('Temporary value');
    title('Temporary Value over time');

    if save
        timestamp = datetime('now', 'Format', 'dd_MM-HH_mm');
        filename = ['Workflows\complete_robot\plots_for_debug\temp_variable\temp_plot-' char(timestamp) '.fig'];
        saveas(gcf, filename);
    end
end
