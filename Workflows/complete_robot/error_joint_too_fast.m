function error_joint_too_fast(str1,str2)

if str1 == "Si"
    string = "Sideways";
elseif str1 == "Sh"
    string = "Shoulder";
else 
    string = "Knee";
end
disp( string + " Joint " + str2 + " is moving too fast!")
end
