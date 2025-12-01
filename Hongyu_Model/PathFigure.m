plot(out.logsout.get('x_robot').Values.Data, out.logsout.get('y_robot').Values.Data,'r')
hold on;
plot(waypoints(:,1),waypoints(:,2))