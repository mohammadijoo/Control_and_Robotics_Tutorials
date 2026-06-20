
function label = classifySystem(cT, iEU, rR)
    th_c = 0.3; th_i = 0.2; th_r = 2;
    if cT < th_c && iEU < th_i && rR == 1
        label = "Automation";
    elseif cT >= th_c && iEU >= th_i && rR >= th_r
        label = "Robot";
    else
        label = "Mechatronic System";
    end
end

disp("CNC line -> " + classifySystem(0.1, 0.05, 1));
disp("Active suspension -> " + classifySystem(0.4, 0.15, 2));
disp("Mobile robot -> " + classifySystem(0.7, 0.6, 5));
      