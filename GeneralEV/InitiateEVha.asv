% ************************************************************************
% This script initiates all parameters, look up tables etc needed for
% simulation of a "parallel" hybrid Simulink model.
%
% The script is subject to continuous development and changes and a good
% understaning of the relation the Simulink program is needed for a useful
% outcome.
%
% Mats Alakula, Lund, September 04 2022.
%
% ************************************************************************
close all
rho_air = 1.2;  % Air density
grav = 9.81;    % Gravitational constant
energy = []; % energy consumption matrix
Pem_max_v = [208000 320000];
Step = [0.5 0.75 1 1.25 1.5];

addpath DriveCycles\
% *** Vehicle Choise *****************************************************
% In this section the main vehicle parameters are selected, for 4 different
% types of vehicles.

for i=1:2
    Vehicle = i;
    Drive_cycle = 2;
    for ii=1:5
        switch Vehicle
            case 1 % Tesla        
                Wbatt = 82*3.6e6;   % [Ws]
                Mv = 1819;         % Vehicle weight [kg]
                rw = 0.24;	        % wheel radius (m)
                Cd = 0.23; 	        % air_resistance Check this! 
                Cr = 0.011;	        % roll resistance Check this!
                Av = 2.22;   % Front area [m2]
                vmax = 233/3.6;     % Maxhastighet [m/s]
               %Pem_max = 324000;   % Traction machine power [W]
                Pem_max = Pem_max_v(1)*Step(ii)
                Number_of_gears = 1;
                Paux = 200;         % Without AC, assumed
        
                % *** Select Drive Cycle from WLTP
                PWR = Pem_max/Mv;
                if PWR<=22,
                    Drive_cycle = 13;
                elseif (PWR>22)&(PWR<=34)
                    Drive_cycle = 14;
                elseif (PWR>34)
                    Drive_cycle = 15;
                end
             case 2 % Hongqi      
                Wbatt = 84*3.6e6;   % [Ws]
                Mv = 2580;         % Vehicle weight [kg]
                rw = 0.267;	        % wheel radius (m)
                Cd = 0.345; 	        % air_resistance Check this! 
                Cr = 0.015;	        % roll resistance Check this!
                Av = 3.48;   % Front area [m2]
                vmax = 200/3.6;     % Maxhastighet [m/s]
                %Pem_max = 320000;   % Traction machine power [W]
                Pem_max = Pem_max_v(2)*Step(ii)
                Number_of_gears = 1;
                Paux = 200;         % Without AC, assumed
        
                % *** Select Drive Cycle from WLTP
                PWR = Pem_max/Mv;
                if PWR<=22,
                    Drive_cycle = 13;
                elseif (PWR>22)&(PWR<=34)
                    Drive_cycle = 14;
                elseif (PWR>34)
                    Drive_cycle = 15;
                end
        end
        
        % *** Load drive cycles **************************************************
        % The following "switch" selects a drive cycle. The data is
        % boiled down to a matrix "cycle" with 6 columns: 1) time [s], 2) speed
        % reference [m/s], 3) position [m], 4) altitude [m], 5) slope [rad] and 6)
        % slide in availability [-]. The "slide in availability" is in some models
        % used to indicathe the presence of an Electric Road.
        
        clear cycle
        switch Drive_cycle
            case 1 % nedc
                load nedc;
                cycle = nedc;
            case 2 % us06;
                load us06;
                cycle = us06;
            case 3 % Bus 1
                load london159;
                cycle = london159;
            case 4 % Bus 2
                load cbr85;
                cycle = cbr85;
            case 5 % Distribution 1
                load Distr1;
                cycle = Distr1;
            case 6 % Long Haul 1
                load LongHaul1;
                cycle = LongHaul1;
            case 7 % Long Haul 2
                load LongHaul2;
                cycle = LongHaul2;
            case 8 % Long Haul 3
                load LongHaul3;
                cycle = LongHaul3;
            case 9 % Long Haul 4
                load LongHaul4;
                cycle = LongHaul4;
            case 10 % ramp
                load Ramp;
                cycle = Ramp;
            case 11 % Landskrona Slide IN Bus
                load LandskronaSI
                cycle = LAcycle;
            case 12 % SwedenRoute
                load SwedenRoute
                cycle = SwedenRoute;
            case 13 % WLTP1 (P[W]/M[kg] <= 22)
                load WLTP1;
                % load('WLTP1') % May work on Mac OS
                cycle = WLTP1;
            case 14 % WLTP2  (22 < P[W]/M[kg] <= 34)
                load WLTP2;
                % load('WLTP2') % May work on Mac OS
                cycle = WLTP2;
            case 15 % WLTP3 (34 < P[W]/M[kg])
                load WLTP3;
                % load('WLTP3') % May work on Mac OS
                cycle = WLTP3;
            case 16 % Tractor
                load plowing
                cycle = tractor;
            case 17 % LyonDistr (up to 20 ton)
                load LyonDistr;
                cycle = LyonDistr;
            case 18 % AceaRegDel; (20-32 ton)
                load AceaRegDel;
                cycle = AceaRegDel;
            case 19 % AceaLongHaul (> 32 ton)
                load AceaLongHaul;
                cycle = AceaLongHaul;
        end
        
        endtime = max(cycle(:,1));
        
        cycle(1:length(cycle),6)=0; % Make sure that no ERS is indicated
        
        
        % ******************** Road and Performance *******************************
        Max_start_slope = 8; % [%]
        Max_curb_hight = 0.065;  %155;  % [m]
        hc=Max_curb_hight;
        Maxle = 0.7*Mv;
        Tw_curb = rw*Maxle*grav/sin(acos(-(hc-rw)/rw));
        Tw_slope = rw*Mv*grav*sin(atan(Max_start_slope/100));
        Tw_roll = Mv*grav*Cr*rw;
        Tw_drag = rw*0.5*Av*Cd*rho_air*vmax^2;
        Tw_acc = Maxle*(100/3.6/7)*rw;
        
        v = [0:vmax/99:vmax];  % [m/s]
        
        v_base_set = 0;
        for i=1:length(v)
            if i==1
                Tmax(i)=Tw_curb+Tw_slope; % Needed to start
            else
                Tmax(i) = min(Tw_acc + Tw_slope + Tw_roll + rw*0.5*Av*Cd*rho_air*v(i)^2,Pem_max/(v(i)/rw)); % Needed to drive and accelerate
                Tdrag(i) = rw*0.5*Av*Cd*rho_air*v(i)^2;
            end
            if (i>3 & Tmax(i)<Tmax(i-1) & v_base_set==0);
                v_base = v(i);
                v_base_set = 1;
            end
        end
        
        % *** Electric machine parameters  ***************************************
        % The following lines set the EM drive performance
        
        fwr = 15000/6000; % Field weakening ratio, i.e. ratio between max speed and base
        %  speed
        wem_min = 0;
        wem_max = 15000*pi/30; % EM max speed
        wem_vmax = 15000*pi/30; % EM Speed at max vehicle speed
        Tem_max = Pem_max/(wem_max/fwr);  % Peak continuous torque
        % The call for "CreateEMmap" is used for the EM in the same way as the call
        % for i "CreateICEmap_special" s used regarding the ICE. See related
        % comment for the ICE above.
        
        [PtoTem,EtaEM,Tem,Wem,Tem_lim,EnergyConsEM] = CreateEMmapII(Pem_max,wem_max,Tem_max,'PMSM');   % 'AM  '
        
        % *** Transmission settings **********************************************
        
        utvx_min = wem_vmax/(vmax/rw);  % "Highest gear";
        utvx_max = Tmax(1)/max(Tem_lim); % "Gear one".
        
        % the following lines create a sequence of gear ratios
        Utvx_vect = zeros(1,Number_of_gears+1);
        Utvx_vect(1,1) = utvx_min;
        Utvx_vect(1,length(Utvx_vect(1,:))) = inf;
        for i=2:Number_of_gears,
            Utvx_vect(1,i) =  Utvx_vect(1,i-1)*(utvx_max/utvx_min)^(1/(Number_of_gears-1));
        end
        
        EtaGEAR = 0.975; % Simplest possible efficiency model of the transmission
        
        gr1 = utvx_max;
        gr2 = utvx_min;
        
        
        figure
        h=get(0,'Screensize');
        set(gcf,'OuterPosition',[min(h(3:4))+10,50,min(h(3:4))/2-10,min(h(3:4))/2-50]);
        clf
        
        plot(v*3.6,Tdrag,'b--','linewidth',1.5); text(85,max(Tw_drag),'Tw drag')
        hold on
        plot(v*3.6,Tw_roll*ones(size(Tmax)),'r--','linewidth',1.5); text(5,max(Tw_roll),'Tw roll')
        plot(v*3.6,Tw_slope*ones(size(Tmax)),'g--','linewidth',1.5); text(5,max(Tw_slope),'Tw slope')
        plot(v*3.6,Tw_curb*ones(size(Tmax)),'m--','linewidth',1.5); text(5,max(Tw_curb),'Tw curb')
        plot(v*3.6,Tw_acc*ones(size(Tmax)),'c--','linewidth',1.5); text(5,max(Tw_acc),'Tw acc')
        plot(v*3.6,Tmax,'*'); text(150,Tmax(1)/1.5*(1.05-length(Utvx_vect(1,:))/10),[num2str(Pem_max/1000) ' [kW]'])
        grid
        
        plot(Wem*rw*3.6/gr1,Tem_lim*gr1,'r'); text(40,Tmax(1)/2*1.05,['Ettan = ' num2str(floor(10*gr1)/10)])
        plot(Wem*rw*3.6/gr2,Tem_lim*gr2,'g'); text(40,Tmax(1)/2*0.95,['Tvåan = ' num2str(floor(10*gr2)/10)])
        
        for i=1:length(Utvx_vect(1,:))-1
            plot(Wem*rw*3.6/Utvx_vect(1,i),Tem_lim*Utvx_vect(1,i),'r'); text(150,Tmax(1)/1.5*(1.05-i/10),['Gear ' num2str(i)  ' = ' num2str(floor(10*Utvx_vect(1,i))/10)])
        end
        
        xlabel('Speed [km/h]')
        ylabel('Wheel torque [Nm]')
        legend('Drag Torque','Roll Torque','Slope torque (8%)',['Curb Torque ' num2str(Max_curb_hight) ' [m]'],'Accel Torque ("0..100 in 7 sec")','Max Wheel Torque')
        
        
        % *** Power Electronics efficiency ***************************************
        % Since the simulation program is design NOT to express Voltages and
        % Currents, only Powers, the simplest form of PEC efficiency mode is used.
        
        EtaPE = 0.97; % PEC efficiency
        
        % *** Battery parameters *************************************************
        % The battery model is a constant voltage in series with a resistance. The
        % electric variables voltage and current is recalculated into termina power
        % and efficiencies. this is done in the call to "CreateBATTmap".
        
        [EtaBATT,Pbatt,Mbatt,Pbatt_max_new,Wbatt_new]=CreateBATTmap(Pem_max,Wbatt);
        SOC_batt_start_value = 90; % Start value of SOC
        SOC_tract_min = 10;
        SOC_tract_max = 90;
        
        % *** Controller parameters **********************************************
        
        Tau_charge = 2;
        
        % Vårat egna skrivna

        sim("GeneralFEVmodel");
        energy(Vehicle, ii) = EnergyMeas.data(end);
    end
end
