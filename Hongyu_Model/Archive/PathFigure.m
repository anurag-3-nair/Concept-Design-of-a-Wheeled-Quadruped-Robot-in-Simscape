plot(waypoints(:,1),waypoints(:,2),'Color', [0.5 0.5 0.5],LineWidth=6)
hold on;
plot(out.logsout.get('x_robot').Values.Data, out.logsout.get('y_robot').Values.Data,'r',LineWidth=2)

