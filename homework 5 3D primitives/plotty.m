function plotty(p,dp,ddp,dddp,title_s)

figure;
sgtitle(title_s);
subplot(2,2,1)
plot3(p(1,:),p(2,:),p(3,:));
hold on;
plot3(p(1,1),p(2,1),p(3,1), '*','Color', 'g');
plot3(p(1,end),p(2,end),p(3,end), '*', 'Color','r');
title('position')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,2)
plot3(dp(1,:),dp(2,:),dp(3,:));
hold on;
plot3(dp(1,1),dp(2,1),dp(3,1), '*','Color', 'g');
plot3(dp(1,end),dp(2,end),dp(3,end), '*', 'Color','r');
title('velocity')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

subplot(2,2,3)
plot3(ddp(1,:),ddp(2,:),ddp(3,:));
hold on;
plot3(ddp(1,1),ddp(2,1),ddp(3,1), '*','Color','g');
plot3(ddp(1,end),ddp(2,end),ddp(3,end), '*','Color', 'r');
title('acceleration')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;


subplot(2,2,4)
plot3(dddp(1,:),dddp(2,:),dddp(3,:));
hold on;
plot3(dddp(1,1),dddp(2,1),dddp(3,1), '*', 'Color', 'g');
plot3(dddp(1,end),dddp(2,end),dddp(3,end), '*','Color',  'r');
title('jerk')
xlabel('x');
ylabel('y');
zlabel('z');
grid on;

end