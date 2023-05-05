% Circular path for cartesian control 
r = 300; % millimeter

t = 12;% total time for one circle path in second
f= 1/t;

omega = 2*pi*f;

tvec = 0:0.05:12;
xpoints = sin(omega*tvec);
ypoints = cos(omega*tvec);
close all
hold on
for ii = 1:size(tvec,2)
    plot(xpoints(ii),ypoints(ii),'o');
    pause(0.05);
end