close all;

if(isfield(out,'signal_from_control'))
    struct_control_of_team = out.signal_from_control;
    struct_control = out.control_variable;
    struct_x = out.x;
    struct_temp = out.temp;

    Joints_struct.FL_Sh = out.FL_Sh;
    Joints_struct.FL_Si = out.FL_Si;
    Joints_struct.FL_Kn = out.FL_Kn;

    Joints_struct.FR_Sh = out.FR_Sh;
    Joints_struct.FR_Si = out.FR_Si;
    Joints_struct.FR_Kn = out.FR_Kn;

    Joints_struct.RL_Sh = out.RL_Sh;
    Joints_struct.RL_Si = out.RL_Si;
    Joints_struct.RL_Kn = out.RL_Kn;

    Joints_struct.RR_Sh = out.RR_Sh;
    Joints_struct.RR_Si = out.RR_Si;
    Joints_struct.RR_Kn = out.RR_Kn;

    %struct_t_FL = out.t_FL;
    %struct_t_FR = out.t_FR;
    %struct_t_BL = out.t_BL;
    %struct_t_BR = out.t_BR;

    save1 = true; % true: Plot saved, false: Plot not saved
    save2 = false;

    %plot_c_over_t(save2, struct_control, struct_control_of_team);
    %plot_x_over_t(save2, struct_x, struct_control);
    plot_temp_over_t(save2,struct_temp);
    %plot_several_signals(save2, struct_t_FL.time, struct_t_FL, struct_t_FR, struct_t_BL, struct_t_BR);
    plot_angles_of_joints(save2, Joints_struct.RL_Sh.time, Joints_struct);
else
    disp('Please run complete_robot.slx')
end