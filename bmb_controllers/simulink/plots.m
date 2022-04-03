% figure;
% plot(out.pitch.time, out.pitch.data);
% xlabel("Time (seconds)");
% ylabel("Pitch (degrees)");

% figure;
% plot(out.roll.time, out.roll.data);
% xlabel("Time (seconds)");
% ylabel("Roll (degrees)");

% figure;
% plot(out.speed.time, out.speed.data);
% xlabel("Time (seconds)");
% ylabel("Speed (m/s)");

% figure;
% plot(out.altitude.time, out.altitude.data);
% xlabel("Time (seconds)");
% ylabel("Altitude (meters)");


figure;
plot(out.yaw.time, out.yaw.data);
xlabel("Time (seconds)");
ylabel("Yaw (degrees)");
