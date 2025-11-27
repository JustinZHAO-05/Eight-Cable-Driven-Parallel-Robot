function [s,v,a]=quintic_trajectory(s0, sT, v0, vT, a0, aT, T, t_vals)

    a0_coef = s0;
    a1_coef = v0;
    a2_coef = a0 / 2.0;
    a3_coef = (20*(sT - s0) - (8*vT + 12*v0)*T - (3*a0 - aT)*T^2) / (2 * T^3);
    a4_coef = (30*(s0 - sT) + (14*vT + 16*v0)*T + (3*a0 - 2*aT)*T^2) / (2 * T^4);
    a5_coef = (12*(sT - s0) - (6*vT + 6*v0)*T - (a0 - aT)*T^2) / (2 * T^5);
    

    s = a0_coef + a1_coef * t_vals + a2_coef * t_vals^2 + a3_coef * t_vals^3 + a4_coef * t_vals^4 + a5_coef * t_vals^5;

    v = a1_coef  + 2 * a2_coef * t_vals + 3 * a3_coef * t_vals^2 + 4 * a4_coef * t_vals^3 + 5 * a5_coef * t_vals^4;

    a = 2 * a2_coef  + 2 * 3 * a3_coef * t_vals + 3 * 4 * a4_coef * t_vals^2 + 4 * 5 * a5_coef * t_vals^3;