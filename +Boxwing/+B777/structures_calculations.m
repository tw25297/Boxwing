



% Main function to calculate geometry and mass of box wing
function [A_cap, t_cap] = structures_calculations(obj, M_array, h_box_array, b_cap_array, sigma_allow)



    A_cap = M_array ./ (sigma_allow .* h_box_array);
    t_cap = A_cap ./ b_cap_array;
    
end
    


