function y = deg0_360t0__180_180(x)
y = x;
for i = 1:length(x)
    if x(i) > 180
       y(i) = x(i) - 360;
    end
end
end