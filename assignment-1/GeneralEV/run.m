energy = [];

Pem_max_v = [208000 320000];
Step = [0.5 0.75 1 1.25 1.5];

for i=1:2
    Vehicle = i;
    for ii=1:5
        Pem_max = Pem_max_v(i)*Step(ii);
        InitiateEVha;
        sim("GeneralEV\GeneralFEVmodel");
        energy(Vehicle, ii) = EnergyMes.data(end);
    end
end