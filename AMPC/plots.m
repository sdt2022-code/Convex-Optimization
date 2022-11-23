% MUST RUN adaptiveMPC.m FIRST
close all

t = get(x).Time;
xdata = get(x).Data;
xdotdata = get(x_dot).Data;
thetadata = get(theta).Data;
thetadotdata = get(theta_dot).Data;
udata = get(u).Data;
ddata = get(d).Data;

n = size(t, 1);

subplot(3,2,1)
xcurve = animatedline('LineWidth', 2, 'Color', 'r');
axis([0 t(n) -1 12])
title("x")
ylabel("Value [m]")
xlabel("t")
yline(10, '--', 'Target');
hold on

subplot(3,2,2)
xdotcurve = animatedline('LineWidth', 2, 'Color', 'r');
title("xdot")
ylabel("Value [m/s]")
xlabel("t")
axis([0 t(n) -4 4])
yline(0, '--', 'Target');
hold on

subplot(3,2,3)
thetacurve = animatedline('LineWidth', 2, 'Color', 'r');
title("theta")
ylabel("Value [rad]")
xlabel("t")
axis([0 t(n) -0.5 0.5])
yline(0, '--', 'Target');
hold on

subplot(3,2,4)
thetadotcurve = animatedline('LineWidth', 2, 'Color', 'r');
title("thetadot")
ylabel("Value [rad/s]")
xlabel("t")
axis([0 t(n) -2 2])
yline(0, '--', 'Target');
hold on

subplot(3,2,5)
ucurve = animatedline('LineWidth', 2, 'Color', 'r');
title("Input Force")
ylabel("Value [N]")
xlabel("t")
axis([0 t(n) -60 60])
yline(50, '--', 'upper');
yline(-50, '--', 'lower');
hold on

subplot(3,2,6)
dcurve = animatedline('LineWidth', 2, 'Color', 'r');
title("Estimate of the Damping Parameter")
ylabel("Estimated Value [1/s]")
xlabel("t")
axis([0 t(n) 2 6])
yline(5, '--', 'True Value');
hold on

pause(3)

for i = 1:10:n
    addpoints(xcurve, t(i), xdata(i));
    addpoints(xdotcurve, t(i), xdotdata(i));
    addpoints(thetacurve, t(i), thetadata(i));
    addpoints(thetadotcurve, t(i), thetadotdata(i));
    addpoints(ucurve, t(i), udata(min(2001,floor(i*2001/n)+1)));
    addpoints(dcurve, t(i), ddata(i));
    drawnow
end