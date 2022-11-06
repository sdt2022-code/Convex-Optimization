function u = LQR_controller(K, y, yd)

u = -K*(y-yd);

end