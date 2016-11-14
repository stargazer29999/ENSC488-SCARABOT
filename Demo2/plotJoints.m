function plotJoints(joint, num)
%Plot Data
figure % new figure
ax1 = subplot(3,1,1); % top subplot
ax2 = subplot(3,1,2); % middle subplot
ax3 = subplot(3,1,3); % bottom subplot

plot(ax1,joint(:,1),joint(:,2))
title(ax1,strcat('Theta ',num))

ylabel(ax1,strcat('Theta ',num))
xlabel(ax1,'time(UNITS??)' )

plot(ax2,joint(:,1),joint(:,3))
ylabel(ax2,'Velocity ')
xlabel(ax2,'time(UNITS??)' )

plot(ax3,joint(:,1),joint(:,4))
ylabel(ax3,'Accelaration')
xlabel(ax3,'time(UNITS??)' )

end
