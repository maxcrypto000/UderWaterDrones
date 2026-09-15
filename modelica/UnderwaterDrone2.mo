model UnderwaterDronePhysics
  parameter Real m = 15.0 "Massa (kg)";
  parameter Real kd = 5.0 "Coefficiente di attrito quadratico dell'acqua";
  
  // --- PARAMETRI BATTERIA ---
  parameter Real battery_max = 100.0 "Capacità massima della batteria (%)";
  parameter Real idle_drain = 0.01 "Consumo passivo (fisso) al secondo (%)";
  parameter Real motor_drain_coeff = 0.05 "Moltiplicatore di consumo per Newton di spinta";
  parameter Real recharge_rate = 10.0 "Velocità di ricarica quando alla base (% al secondo)";
  parameter Real base_tolerance = 10.0 "Raggio di tolleranza della base in metri";
  
  // Coordinate di start modificabili da C (Evaluate=false)
  parameter Real start_x = 0.0 annotation(Evaluate=false);
  parameter Real start_y = 0.0 annotation(Evaluate=false);
  parameter Real start_z = 0.0 annotation(Evaluate=false);
  
  // OUTPUTS
  output Real x(start=start_x, fixed=true);
  output Real y(start=start_y, fixed=true);
  output Real z(start=start_z, fixed=true);
  output Real vx(start=0.0, fixed=true);
  output Real vy(start=0.0, fixed=true);
  output Real vz(start=0.0, fixed=true);
  output Real battery(start=battery_max, fixed=true) "Livello batteria 0-100"; // NUOVO OUTPUT

  // INPUTS
  input Real ux;
  input Real uy;
  input Real uz;
  
  Real v_norm "Norma della velocita per attrito";
  Real u_norm "Norma della spinta per calcolo consumi";
  Real distance_to_base "Distanza euclidea dal punto di spawn";
  Real net_charge_rate "Rateo di carica/scarica al secondo";

equation
  // 1. Calcoli vettoriali
  v_norm = sqrt(vx^2 + vy^2 + vz^2);
  u_norm = sqrt(ux^2 + uy^2 + uz^2);
  
  // 2. Controllo della distanza dalla base
  distance_to_base = sqrt((x - start_x)^2 + (y - start_y)^2 + (z - start_z)^2);
  
  // 3. Logica di Carica e Scarica
  // Se è vicino alla base si ricarica, altrimenti consuma (fisso + proporzionale ai motori)
  net_charge_rate = if distance_to_base <= base_tolerance then recharge_rate 
                    else -(idle_drain + motor_drain_coeff * u_norm);
  
  // 4. Derivata della batteria (con limiti fisici)
  // Impedisce alla batteria di caricarsi oltre il massimo o di andare in negativo
  der(battery) = if (battery >= battery_max and net_charge_rate > 0) then 0.0 
                 else if (battery <= 0.0 and net_charge_rate < 0) then 0.0 
                 else net_charge_rate;
  
  // 5. Cinematica
  der(x) = vx;
  der(y) = vy;
  der(z) = vz;
  
  // 6. Dinamica
  der(vx) = (ux - kd * vx * v_norm) / m;
  der(vy) = (uy - kd * vy * v_norm) / m;
  der(vz) = (uz - kd * vz * v_norm) / m;
  
end UnderwaterDronePhysics;
