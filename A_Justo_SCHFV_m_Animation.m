clc
%close all
%acc1=0;acc2=0;ColorAV1=[1 1 1]*0.5;ColorAV2=[1 1 1]*0.5;n1=-10;
% Parameters
Width_vehicles = 1.9*ones(N,1);

% Line track
thetavec = linspace(0,2*pi);

% Video
vid = VideoWriter('sims','MPEG-4');
vid.FrameRate = 30;
myVideo.Quality = 1080;

% Animation
figure(4)
set(figure(4), 'Position', get(0, 'Screensize'));hold on;
subplot(2,11,[1 5 12 16],'Color','k')
Text = text(-14,-1.2*R,['Time: ', num2str(time(1)), ' s'],'FontSize',20);
r1=R-2;r2=R+2;xf=0;Xf=0;yf=0;Yf=0;
x = xf + r1*cos(thetavec);
y = yf + r1*sin(thetavec);
X = Xf + r2*cos(thetavec);
Y = Yf + r2*sin(thetavec);
patch([x X],[y Y],'k','linestyle','non');
title(['Traffic flow in a circular track of ' num2str(d) ' m'],'FontSize',20);
hold on
logo = imread('IOC.jpg'); image([-0.5*R 0.5*R],[0.5*R -0.5*R],logo);
axis([-1.07*R 1.07*R -1.07*R 1.07*R]);
axis off; hold on; axis equal;

% Initializing vehicles
X_1 = []; X_2 = []; X_3 = [];
Y_1 = []; Y_2 = []; Y_3 = [];
for k = 1:N
    Front = sims_th(1,k);
    Rear  = Front + sims_L(1,k)/R;
    linX = cos(linspace(Front,Rear));
    linY = sin(linspace(Front,Rear));
    x = xf + 0.5*(r1+r2-Width_vehicles(k))*linX;
    X = Xf + 0.5*(r1+r2+Width_vehicles(k))*linX; X = fliplr(X);
    y = yf + 0.5*(r1+r2-Width_vehicles(k))*linY;
    Y = Yf + 0.5*(r1+r2+Width_vehicles(k))*linY; Y = fliplr(Y);
    Xk = [x X]'; Yk = [y Y]';
    if k==acc1
        X_1 = [X_1,Xk];
        Y_1 = [Y_1,Yk];
    elseif k==acc2
        X_2 = [X_2,Xk];
        Y_2 = [Y_2,Yk];
    else
        X_3 = [X_3,Xk];
        Y_3 = [Y_3,Yk];
    end
end
p1 = patch(X_1,Y_1,ColorAV1);
p2 = patch(X_2,Y_2,ColorAV2);
p3 = patch(X_3,Y_3,ColorHDV);

figure(4)
% Positions
subplot(2,11,[7 11])
h = animatedline;
for i=1:N
    if i==acc1 && acc2~=0
        col=ColorAV1; lw=2;
    elseif i==acc2
        col=ColorAV2; lw=2;
    elseif i==acc1+n1 && acc2==0
        col=ColorVH1; lw=2;
    elseif i==acc1 && acc2==0
        col=ColorVA1; lw=2;
    else
        col=ColorHDV; lw=1;
    end
    [aux,loc]=findpeaks(sims_th(:,i));
    if length(loc)>1
        ind2=loc(1)-1;
        plot(time(1:ind2),sims_th(1:ind2,i)*R,'Color',col,'Linewidth',lw)
        hold on
        for j=1:length(loc)-1
            ind1=loc(j)+1;
            ind2=loc(j+1)-1;
            plot(time(ind1:ind2),sims_th(ind1:ind2,i)*R,...
                'Color',col,'Linewidth',lw)
        end
        ind1=loc(end)+1;
        plot(time(ind1:end),sims_th(ind1:end,i)*R,...
            'Color',col,'Linewidth',lw)
    end
end
axis([0 tfinal 0 d])
t1 = line([0 0], ylim);
hold on;
title('Position and speed of vehicles','FontSize',20);
ax = gca; ax.FontSize = 20; grid on
xlabel('Time [s]','FontSize',20);
ylabel('Position on track [m]','FontSize',20)
% Speeds
figure(4)
subplot(2,11,[18 22])
for i=1:N
    if i==acc1 && acc2~=0
        col=ColorAV1; lw=2;
        va1 = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc2
        col=ColorAV2; lw=2;
        va2 = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc1+n1 && acc2==0
        col=ColorVH1; lw=2;
        vh = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    elseif i==acc1 && acc2==0
        col=ColorVA1; lw=2;
        va = plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    else
        col=ColorHDV; lw=1;
        plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw);
        hold on
    end
    plot(time,sims_v(:,i)*3.6,'Color',col,'Linewidth',lw)
    hold on
end
av = plot(time,sims_vav*3.6,':k','Linewidth',1);
vr = plot(time,sims_v_r*3.6,'r');
t2 = line([0 0], ylim);
% hold on;
% legend([vr,va1,va2,av],{'Reference speed','Autonomous vehicle #1',...
%     'Autonomous vehicle #2','Average speed'},'Location','SouthEast','Fontsize',14);
legend([vr,va,vh,av],{'Reference speed','Autonomous vehicle',...
    'Last fleet vehicle','Average speed'},'Location','SouthEast','Fontsize',16);
legend(av,{'Average speed'},'Location','SouthEast','Fontsize',16);
%title('Speed of vehicles','FontSize',20);
ax = gca; ax.FontSize = 20; grid on
xlabel('Time [s]','FontSize',20);
ylabel('Speed [km/h]','FontSize',20);

%% Update video
Counter = 0;
At_video=30;
for i=1:At_video:length(time)
    X_1 = []; X_2 = []; X_3 = [];
	Y_1 = []; Y_2 = []; Y_3 = [];   
    for k = 1:N
        Front = sims_th(i,k);
        Rear  = Front + sims_L(i,k)/R;
        linX = cos(linspace(Front,Rear));
        linY = sin(linspace(Front,Rear));
        x = xf + 0.5*(r1+r2-Width_vehicles(k))*linX;
        X = Xf + 0.5*(r1+r2+Width_vehicles(k))*linX; X = fliplr(X);
        y = yf + 0.5*(r1+r2-Width_vehicles(k))*linY;
        Y = Yf + 0.5*(r1+r2+Width_vehicles(k))*linY; Y = fliplr(Y);
        Xk = [x X]'; Yk = [y Y]';
        if k==acc1
            X_1 = [X_1,Xk];
            Y_1 = [Y_1,Yk];
        elseif k==acc2
            X_2 = [X_2,Xk];
            Y_2 = [Y_2,Yk];
        else
            X_3 = [X_3,Xk];
            Y_3 = [Y_3,Yk];
        end
    end
    set(p1,'XData',X_1,'YData',Y_1);
    set(p2,'XData',X_2,'YData',Y_2);
    set(p3,'XData',X_3,'YData',Y_3);
    set(t1,'XData',[time(i) time(i)]);
    set(t2,'XData',[time(i) time(i)]);
    set(Text,'String',['Time: ',num2str(time(i),'% 3.0f'),' s'],'FontSize',20);
    open(vid);
    f = getframe(gcf);
	writeVideo(vid,f);
end
close(vid);