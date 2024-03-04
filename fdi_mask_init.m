%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%     FDI PART     %%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%                  %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%                             %%%%%%%%%
%%%%%%                             %%%%%%%%%
%%%%%% y=Gain_Matrix * x + ksi + F %%%%%%%%%
%%%%%%                             %%%%%%%%%
%%%%%%                             %%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

airspeed0=5; %189;  %v_x(0) can be taken v_x_ave as well
beta0=1;
alpha0=2.5;
%fault0 = evalin('base','fault0');
%Sigma=0.001;
Ts=Tsample; %Ts_record;


fdi_number_of_sensors=3;                            %number of sensors

%%%%%%%%%%%%%%%      INPUTS    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%fdi_fault_check=1;                                  %Fault check,1:fault,0:no fault

%AIRS
% Sensor 1 - Fault
fdi_fault_type_check_1_airs=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_1_airs=3; %3; %airspeed0*0.01; %1.01;                                %value of fault, f1
fdi_start_time_1_airs=2000; %45; %500;                               %Start of fault (sec)
fdi_duration_1_airs=550; %1000;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_1_airs=fdi_fault_value_1_airs;
fdi_gain_matrix_1_airs=1;

%Sensor 2 - Fault
fdi_fault_type_check_2_airs=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_2_airs=0; %3; %beta0*0.1;                                %value of fault, f2
fdi_start_time_2_airs=0; %500; %0; %200;                               %Start of fault (sec)
fdi_duration_2_airs=0; %1000;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_2_airs=fdi_fault_value_2_airs;
fdi_gain_matrix_2_airs=1;

%Sensor 3 - Fault
fdi_fault_type_check_3_airs=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_3_airs=0; %alpha0*0.1;                                %value of fault, f3
fdi_start_time_3_airs=0; %700;                               %Start of fault (sec)
fdi_duration_3_airs=0; %250;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_3_airs=fdi_fault_value_3_airs;
fdi_gain_matrix_3_airs=1;

%%%%%%%%%%%%%%%%%
%BETA

% Sensor 1 - Fault
fdi_fault_type_check_1_beta=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_1_beta=1; %airspeed0*0.01; %1.01;                                %value of fault, f1
fdi_start_time_1_beta=1125; %500;                               %Start of fault (sec)
fdi_duration_1_beta=1000;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_1_beta=fdi_fault_value_1_beta;
fdi_gain_matrix_1_beta=1;

%Sensor 2 - Fault
fdi_fault_type_check_2_beta=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_2_beta=0; %1; %beta0*0.1;                                %value of fault, f2
fdi_start_time_2_beta=0; %1125;                               %Start of fault (sec)
fdi_duration_2_beta=0; %1000;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_2_beta=fdi_fault_value_2_beta;
fdi_gain_matrix_2_beta=1;

%Sensor 3 - Fault
fdi_fault_type_check_3_beta=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_3_beta=0; %alpha0*0.1;                                %value of fault, f3
fdi_start_time_3_beta=0; %700;                               %Start of fault (sec)
fdi_duration_3_beta=0; %250;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_3_beta=fdi_fault_value_3_beta;
fdi_gain_matrix_3_beta=1;

%%%%%%%%%%%%%%%%%
%ALPHA

% Sensor 1 - Fault
fdi_fault_type_check_1_alpha=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_1_alpha=0; %airspeed0*0.01; %1.01;                                %value of fault, f1
fdi_start_time_1_alpha=0; %500;                               %Start of fault (sec)
fdi_duration_1_alpha=0;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_1_alpha=fdi_fault_value_1_alpha;
fdi_gain_matrix_1_alpha=1;

%Sensor 2 - Fault
fdi_fault_type_check_2_alpha=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_2_alpha=0; %beta0*0.1;                                %value of fault, f2
fdi_start_time_2_alpha=0; %200;                               %Start of fault (sec)
fdi_duration_2_alpha=0;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_2_alpha=fdi_fault_value_2_alpha;
fdi_gain_matrix_2_alpha=1;

%Sensor 3 - Fault
fdi_fault_type_check_3_alpha=0;                           %fault type check,1:Gain,0:fixed
fdi_fault_value_3_alpha=1; %alpha0*0.1;                                %value of fault, f3
fdi_start_time_3_alpha=1750;                               %Start of fault (sec)
fdi_duration_3_alpha=1000; %250;                                 %Duration of fault (sec) for no fault in this sensor> duration=0
fdi_fixed_matix_3_alpha=fdi_fault_value_3_alpha;
fdi_gain_matrix_3_alpha=1;

fdi_alpha=0.01;                                 %Significance level of ALARM (%)
fdi_alarm_thrsld=9.2103; %chi2inv(1-fdi_alpha,2); %9.2103       %alarm threshold wrt chi sqr 2nd order

%
%
% NOISE VARIANCE
% FROM ULB STUDY, for 176 = 0.031 var (0.176 std)
% FROM WIND NOISE STUDY, 190= 0.04 var (0.2 std)
% 0= 1.2250e-05  (0.0035 std)
% 0.1005= 1.9360e-05   (0.0044 std)

Noise_airs = 1*0.01; %NOMINAL 0.2; %(Sigma*airspeed0)^2*1;
Noise_beta = 0.002; %0.04; %degree %0.002; %(Sigma*beta0)^2*1;
Noise_alpha = 0.002; %0.06; %degree %0.002; %(Sigma*alpha0)^2*1;
fdi_noise_var_airs=Noise_airs^2;  %HES 2019
fdi_noise_var_beta=Noise_beta;
fdi_noise_var_alpha=Noise_alpha;

fdi_R_airs=fdi_noise_var_airs * eye(3);                   %matrix R
fdi_R_beta=fdi_noise_var_beta * eye(3);
fdi_R_alpha=fdi_noise_var_alpha * eye(3);

fdi_V=(null([1 1 1]))';                         %matrix V

fdi_SmT_airs= (fdi_V * fdi_R_airs * fdi_V')^(-1/2);        %matrix S with minus tranpose
fdi_SmT_beta= (fdi_V * fdi_R_beta * fdi_V')^(-1/2);
fdi_SmT_alpha= (fdi_V * fdi_R_alpha * fdi_V')^(-1/2);


fdi_mu_1_airs=fdi_SmT_airs*fdi_V*[1;0;0];
fdi_mu_2_airs=fdi_SmT_airs*fdi_V*[0;1;0];
fdi_mu_3_airs=fdi_SmT_airs*fdi_V*[0;0;1];

fdi_mu_1_beta=fdi_SmT_beta*fdi_V*[1;0;0];
fdi_mu_2_beta=fdi_SmT_beta*fdi_V*[0;1;0];
fdi_mu_3_beta=fdi_SmT_beta*fdi_V*[0;0;1];

fdi_mu_1_alpha=fdi_SmT_alpha*fdi_V*[1;0;0];
fdi_mu_2_alpha=fdi_SmT_alpha*fdi_V*[0;1;0];
fdi_mu_3_alpha=fdi_SmT_alpha*fdi_V*[0;0;1];


fdi_muNmu_1_airs=fdi_mu_1_airs/norm(fdi_mu_1_airs);
fdi_muNmu_2_airs=fdi_mu_2_airs/norm(fdi_mu_2_airs);
fdi_muNmu_3_airs=fdi_mu_3_airs/norm(fdi_mu_3_airs);

fdi_muNmu_1_beta=fdi_mu_1_beta/norm(fdi_mu_1_beta);
fdi_muNmu_2_beta=fdi_mu_2_beta/norm(fdi_mu_2_beta);
fdi_muNmu_3_beta=fdi_mu_3_beta/norm(fdi_mu_3_beta);

fdi_muNmu_1_alpha=fdi_mu_1_alpha/norm(fdi_mu_1_alpha);
fdi_muNmu_2_alpha=fdi_mu_2_alpha/norm(fdi_mu_2_alpha);
fdi_muNmu_3_alpha=fdi_mu_3_alpha/norm(fdi_mu_3_alpha);


fdi_seed_airs=ceil(rand(1,fdi_number_of_sensors)*1000);  %seeds for random numbers
fdi_seed_beta=ceil(rand(1,fdi_number_of_sensors)*1000);  %seeds for random numbers
fdi_seed_alpha=ceil(rand(1,fdi_number_of_sensors)*1000);  %seeds for random numbers


% fdi_total_fault_duration=(fdi_duration_1+fdi_duration_2+fdi_duration_3)*(1/Ts);
% 
% if fdi_fault_check==1
%     
%     if fdi_duration_1~=0
%         
%     if fdi_fault_type_check_1==0;
%         fdi_fixed_matix_1=fdi_fault_value_1;
%         fdi_gain_matrix_1=1;
%     elseif fdi_fault_type_check_1==1;
%         fdi_fixed_matix_1=0;
%         fdi_gain_matrix_1=fdi_fault_value_1;
%     end
%     
%     elseif fdi_duration_1==0
%         fdi_fixed_matix_1=0;
%         fdi_gain_matrix_1=1;
%     end
% 
% 
%     if fdi_duration_2~=0
% 
%     if fdi_fault_type_check_2==0;
%         fdi_fixed_matix_2=fdi_fault_value_2;
%         fdi_gain_matrix_2=1;
%     elseif fdi_fault_type_check_2==1;
%         fdi_fixed_matix_2=0;
%         fdi_gain_matrix_2=fdi_fault_value_2;
%     end
% 
%     elseif fdi_duration_2==0
%         fdi_fixed_matix_2=0;
%         fdi_gain_matrix_2=1;
%     end
% 
%     if fdi_duration_3~=0
% 
%     if fdi_fault_type_check_3==0;
%         fdi_fixed_matix_3=fdi_fault_value_3;
%         fdi_gain_matrix_3=1;
%     elseif fdi_fault_type_check_3==1;
%         fdi_fixed_matix_3=0;
%         fdi_gain_matrix_3=fdi_fault_value_3;
%     end
%     
%     elseif fdi_duration_3==0
%         fdi_fixed_matix_3=0;
%         fdi_gain_matrix_3=1;
%     end
%     
%     fdi_total_fault_duration=(fdi_duration_1+fdi_duration_2+fdi_duration_3)*(1/Ts);
% 
%     
% elseif fdi_fault_check==0
%     fdi_gain_matrix_1=1;fdi_gain_matrix_2=1;fdi_gain_matrix_3=1;
%     fdi_fixed_matix_1=0;fdi_fixed_matix_2=0;fdi_fixed_matix_3=0;
%     
%     fdi_start_time_1=0;fdi_start_time_2=0;fdi_start_time_3=0;
%     fdi_duration_1=0;fdi_duration_2=0;fdi_duration_3=0;
%     
%     fdi_total_fault_duration=0;
% 
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%