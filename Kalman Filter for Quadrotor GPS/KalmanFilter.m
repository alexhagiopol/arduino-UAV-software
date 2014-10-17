clear
clc
close all

scale = 1/10000000; %scale to get to the real GPS data points form the saved Arduino data
init_length = 1;
dataFilePath = 'Data/loop_walk.csv';
stationaryDataFilePath = 'Data/Stationary_Data_10_02_14.csv';

indT = 10;
indGPSLat = 8;
indGPSLong = 9;
data = csvread(dataFilePath);

%get index of the non default GPS data points (default outputed when KalmanNoData() was called)
good_data_index = find(data(:,indGPSLat)~=-1);

delta_T = [0; data(2:end,indT)-data(1:end-1,indT)]/1000;

%scale the saved GPS down to it's real value
data = scale*data(:,indGPSLat:indGPSLat+1);

%converting GPS data into meters
cosLat = cos(data(1,1)*pi/180);
Rearth = 6378137;
data(:,1) = (data(1:end,1)-data(1,1))*pi/180*Rearth;
data(:,2) = (data(1:end,2)-data(1,2))*pi/180*Rearth*cosLat;

%computing the measurement noise covariance
stationaryData = csvread(stationaryDataFilePath);
stationaryData = stationaryData(:,1:2);
cosLatStationary = cos(stationaryData(1,1)*pi/180);
stationaryData(:,1) = (stationaryData(1:end,1)-stationaryData(1,1))*pi/180*Rearth;
stationaryData(:,2) = (stationaryData(1:end,2)-stationaryData(1,2))*pi/180*Rearth*cosLatStationary;

%plotting the measurement noise
%figure; plot(stationaryData(:,1), 'r'); hold on; plot(stationaryData(:,2),'--b');legend('latitude', 'longitude');xlabel('measurement count'); ylabel('averaged-out distance (in m)');set(findall(gcf,'type','text'),'FontSize',17)

RcovarianceMatrix = cov(stationaryData); %this is measurement noise covariance; same as measurement covariance because we assume the mean noise is 0  
QcovarianceMatrix = [0.01, 0, 0, 0;
                     0, 0.01, 0, 0;
                     0, 0, 0.001, 0;
                     0, 0, 0, 0.001]; %set process noise covariance equal to measurement noise covariance 

%Initialize variables
Amatrix = [1,0,delta_T(init_length+1),0;
           0,1,0,delta_T(init_length+1);
           0,0,1,0     ;
           0,0,0,1     ;];
       
lat = data(init_length,1);
long = data(init_length,2);

Hmatrix = [1,0,0,0;
           0,1,0,0];
Xstate = [lat, long, 0, 0];

PerrorCovariance = [0.001, 0, 0, 0;
                    0, 0.001, 0, 0;
                    0, 0, 0.02, 0;
                    0, 0, 0, 0.02];
                


[rows, cols] = size(data);
estimateStorage = zeros(init_length, 4);

for i = init_length+1:rows
    %Prediction Step
    Amatrix = [1,0,delta_T(i),0;
           0,1,0,delta_T(i);
           0,0,1,0     ;
           0,0,0,1     ;];
    
    nextXstateEstimate = Amatrix*Xstate';
    
    PerrorCovarianceEstimate = Amatrix*PerrorCovariance*Amatrix' + QcovarianceMatrix; 
    
    if(sum(good_data_index==i)>0) %%if there was a measurement for this data
        %Correction Step
        KalmanGain = PerrorCovarianceEstimate*Hmatrix'/(Hmatrix*PerrorCovarianceEstimate*Hmatrix' + RcovarianceMatrix);
        Zk = [data(i,1),data(i,2)];
        Xstate = (nextXstateEstimate + KalmanGain*(Zk' - Hmatrix*nextXstateEstimate))'
        PerrorCovariance = (eye(4) - KalmanGain*Hmatrix)*PerrorCovarianceEstimate;
    else
        Xstate = nextXstateEstimate';
        PerrorCovariance = PerrorCovarianceEstimate;
    end
    
    estimateStorage = [estimateStorage; Xstate]; %store data 
end

%retieve original GPS data and online KF results
GPS_data = csvread(dataFilePath);

OnlineKF = GPS_data(:,2:3);
GPS_data = GPS_data(:,indGPSLat:indGPSLat+1)*scale;


GPS_data(:,1) = (GPS_data(1:end,1)-GPS_data(1,1))*pi/180*Rearth;
GPS_data(:,2) = (GPS_data(1:end,2)-GPS_data(1,2))*pi/180*Rearth*cosLat;

%GPS_data = GPS_data(1:end,1:2);

figure
%plot original GPS data
plot(OnlineKF(:,2),OnlineKF(:,1), 'm*', 'Markersize', 5.7);
title('')
xlabel('longitude from origin (in m)');
ylabel('latitude from origin (in m)');

hold on

%plot Kalman filtered GPS data
scatter(GPS_data(good_data_index,2),GPS_data(good_data_index,1),40*ones(size(GPS_data(good_data_index,1))), 'k+');
legend('online KF','raw data');
hold off

figure;
subplot(1,2,1)
%scatter(1:size(estimateStorage(:,1)),estimateStorage(:,1),'ob');
hold on
scatter(good_data_index,GPS_data(good_data_index,1), '+k');
plot(1:size(OnlineKF(:,1)), OnlineKF(:,1),'m*', 'Markersize', 4)
hold off
title('latitude')
xlabel('measurement count')
ylabel('latitude (in m)')
subplot(1,2,2)
%scatter(1:size(estimateStorage(:,2)),estimateStorage(:,2),'ob');
hold on
scatter(good_data_index, GPS_data(good_data_index,2), '+k');
plot(1:size(OnlineKF(:,2)), OnlineKF(:,2),'m*', 'Markersize', 4)
hold off
title('longitude')
xlabel('measurement count')
ylabel('latitude (in m)')
legend('raw data', 'online KF')

total_estimate_distance = sqrt(estimateStorage(:,1).*estimateStorage(:,1) + estimateStorage(:,2).*estimateStorage(:,2));
total_GPS_distance = sqrt(GPS_data(:,1).*GPS_data(:,1) + GPS_data(:,2).*GPS_data(:,2));
total_online_distance = sqrt(OnlineKF(:,1).*OnlineKF(:,1) + OnlineKF(:,2).*OnlineKF(:,2));

figure;
plot(total_online_distance,'*m','MarkerSize',4);
hold on
scatter(good_data_index, total_GPS_distance(good_data_index), 200*ones(size(good_data_index)), '+k');
hold off
legend('online KF', 'raw data');
title('total euclidian distance from origin')
xlabel('measurement count')
ylabel('distance (in m)')

% rawMeterDistances = meterDist(GPS_data/scale);
% kalmanMeterDistances = meterDist(estimateStorage(:,1:2)/scale);
% 
% %plot original GPS data
% figure;
% plot(rawMeterDistances, 'r');
% hold on
% 
% %plot Kalman filtered GPS data
% plot(kalmanMeterDistances, 'b');
% legend('raw data','filtered data');
% hold off
 
