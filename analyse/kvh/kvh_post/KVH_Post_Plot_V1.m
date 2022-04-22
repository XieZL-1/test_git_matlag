set(groot,'defaultLineLineWidth',1)

f_figure(1,'位置')
    subplot(2,1,1)
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Longitude)/57.3*6400000,'DisplayName','Longitude')
    xlabel('时间 s')
    ylabel('Latitude °')
    subplot(2,1,2)
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Latitude)/57.3*6400000,'DisplayName','Latitude')
    xlabel('时间 s')
    ylabel('Longitude °')
    legend
    grid on
f_figure(2,'速度')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.VEast),'DisplayName','Longitude')
    plot( KVH_Post.LocalTime, (KVH_Post.VNorth),'DisplayName','Latitude')
    xlabel('时间 s')
    ylabel('Vel m/s')
    legend
    grid on
f_figure(3,'Att')
    hold on
    subplot(3,1,1)
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Roll),'DisplayName','Longitude')
    subplot(3,1,2)
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Pitch),'DisplayName','Latitude')
    subplot(3,1,3)
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.Heading),'DisplayName','Latitude')
    legend
    grid on
    
f_figure(4,'加速度')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.AccBdyX),'DisplayName','Longitude')
    plot( KVH_Post.LocalTime, (KVH_Post.AccBdyY),'DisplayName','Latitude')
    plot( KVH_Post.LocalTime, (KVH_Post.AccBdyZ),'DisplayName','Latitude')
    legend
    grid on

f_figure(5,'角速度')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.AngRateX),'DisplayName','X')
    plot( KVH_Post.LocalTime, (KVH_Post.AngRateY),'DisplayName','Y')
    plot( KVH_Post.LocalTime, (KVH_Post.AngRateZ),'DisplayName','Z')
    legend
    grid on
    
f_figure(6,'时间')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.LocalTime),'DisplayName','Longitude')
f_figure(7,'GyroBias')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.GyroDriftX),'DisplayName','X')
    plot( KVH_Post.LocalTime, (KVH_Post.GyroDriftY),'DisplayName','Y')
    plot( KVH_Post.LocalTime, (KVH_Post.GyroDriftZ),'DisplayName','Z')
    legend
    grid on
    xlabel('时间 s')
    ylabel('deg/s')
f_figure(8,'ACCBias')
    hold on
    plot( KVH_Post.LocalTime, (KVH_Post.AccBiasX),'DisplayName','X')
    plot( KVH_Post.LocalTime, (KVH_Post.AccBiasY),'DisplayName','Y')
    plot( KVH_Post.LocalTime, (KVH_Post.AccBiasZ),'DisplayName','Z')
    legend
    grid on
    xlabel('时间 s')
    ylabel('m/s^2')
    
f_figure(10000,'位置')
    hold on
    plot( KVH_Post.Longitude, KVH_Post.Latitude, '.','DisplayName','位置')
    xlabel('Latitude °')
    ylabel('Longitude °')
    legend
    grid on
