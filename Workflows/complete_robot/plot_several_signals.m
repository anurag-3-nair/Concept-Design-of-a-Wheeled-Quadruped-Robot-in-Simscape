function plot_several_signals(save,time, struct1, struct2, struct3, struct4)
    data1 = struct1.signals.values;
    data2 = struct2.signals.values;
    data3 = struct3.signals.values;
    data4 = struct4.signals.values;
    
    figure;
    plot(time, data1);
    hold on;
    plot(time, data2);
    plot(time, data3);
    plot(time, data4);    
    xlabel('Time');
    ylabel('Several signals value');
    title('Several signals over time');

    if save
        timestamp = datetime('now', 'Format', 'dd_MM-HH_mm');
        filename = ['Workflows\complete_robot\plots_for_debug\several_signals\temp_plot-' char(timestamp) '.fig'];
        saveas(gcf, filename);
    end
    hold off
end