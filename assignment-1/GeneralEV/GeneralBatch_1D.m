clear all
close all

TractionPower_vector = [100 129 158 188 217 246 275 304 333 363 392 421 450];   % Traction machine power [kW]

Charging_vector = [900 1000 1100 1200 1300 1400 1500 1600 1700 2000 2500 3000 3500;    % Fast charging service weight [kg]
    20 29 38 48 57 66 75 84 93 103 112 121 130; % Fast charging battery capacity [kWh]
    775 817 860 902 945 987 1030 1073 1115 1358 1800 2243 2685; % ERS service weight [kg]
    8 12 15 19 23 26 30 34 37 41 45 48 52]; % ERS battery capacity

for loop1 = 1:2,
    for loop2 = 1:length(Charging_vector);
        Vehicle = 1;
        close all
        Pem_max = TractionPower_vector(loop2)*1000;
        if loop1==1,
            Mv =  Charging_vector(loop1, loop2);
            Wbatt = Charging_vector(loop1+1,loop2)*3.6e6;
        elseif loop1==2,
            Mv =  Charging_vector(loop1+1, loop2);
            Wbatt = Charging_vector(loop1+2,loop2)*3.6e6;
        else
            'Fault'
        end
        [Mv Pem_max Wbatt/3.6e6]
        InitiateEV;
        sim('GeneralFEVmodel')
        ECresult(loop1,loop2)=EC(length(EC));
    end
end
    
