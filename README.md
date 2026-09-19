# Underwater Drone Swarm: Autonomous Control with FMI & Neural Networks

Questo progetto implementa e simula il controllo autonomo per uno sciame di droni sottomarini. L'architettura è basata su:
- **OpenModelica / FMI 2.0**: Il modello fisico del drone (cinematica, dinamica e consumi energetici) esportato come *Functional Mock-up Unit* (FMU).
- **Intelligenza Artificiale**: Una Rete Neurale Feed-Forward addestrata con un algoritmo di *Evolution Strategies* per evitare ostacoli e raggiungere i target in ambienti generati proceduralmente (raycasting a 64 raggi).
- **Mission System**: Una Macchina a Stati Finiti in C che utilizza la rete pre-addestrata per compiere missioni a durata indefinita gestendo automaticamente i cicli di ricarica della batteria.

## 🛠️ Prerequisiti

Per compilare ed eseguire il progetto, sono necessari:
- Un compilatore C (es. MSVC su Windows o GCC/Clang su Linux/Mac).
- [CMake](https://cmake.org/) (versione 3.10 o superiore).
- **Python 3** (con le librerie matplotlib e 
umpy installate per eseguire le visualizzazioni).
- L'eseguibile richiede la cartella fmu/ contenente il file UnderwaterDronePhysics.fmu.

## ⚙️ Compilazione

Il progetto utilizza **CMake** per gestire la build. Dalla cartella principale del progetto, esegui i seguenti comandi nel terminale:

`powershell
mkdir build
cd build
cmake ..
cmake --build .
`

Questo processo compilerà il codice C collegandolo alla libreria *FMILibrary* e genererà l'eseguibile DroneTest.exe (su Windows) o DroneTest (su Linux/Mac).

---

## 🚀 Guida all'Utilizzo

L'eseguibile principale offre 3 modalità operative basate sugli argomenti da riga di comando.
*Assicurati di lanciare l'eseguibile dalla directory build/ (oppure assicurati che il percorso dell'FMU sia corretto rispetto a dove lanci il comando).*

### 1. Fase di Addestramento (Training)
**Attenzione**: la fase di addestramento richiede circa 5 ore su portatile; per provare il sistema è consigliato usare la rete pre-addestrata.
`powershell
.\build\DroneTest.exe
`
Se lanciato senza alcun parametro, il programma avvierà la fase di **Training**.
- L'algoritmo di Evolution Strategy proverà iterativamente piccole variazioni (jitter) sui pesi della Rete Neurale valutandone le performance (Fitness) nell'evitare ostacoli e raggiungere l'obiettivo.
- Al termine delle generazioni, il modello migliore verrà salvato nel file binario best_model.bin.
- Verranno generati i file telemetry.csv (l'andamento fisico della generazione migliore) e environment.csv (i confini e gli ostacoli della mappa).

### 2. Fase di Test (Testing)
`powershell
.\build\DroneTest.exe test
`
Questa modalità permette di testare la Rete Neurale pre-addestrata su uno scenario generato casualmente.
- Il programma caricherà i pesi da best_model.bin.
- I droni cercheranno di raggiungere l'obiettivo schivando le nuove montagne generate proceduralmente.
- **Nota**: In questa fase la logica della batteria e la ricarica autonoma non sono attive; serve puramente per testare l'evitamento degli ostacoli (Obstacle Avoidance).

### 3. Fase di Missione Dinamica (Mission)
`powershell
.\build\DroneTest.exe mission [num_drones] [num_obstacles] [seed]
`
Questa modalità lancia il simulatore avanzato governato da una **Macchina a Stati Finiti**.
- Utilizza best_model.bin per guidare i droni.
- Ai droni viene assegnato un target dinamicamente. Appena raggiunto, ne viene generato uno nuovo entro un raggio specificato (I_RADIUS).
- Puoi personalizzare la missione passando opzionalmente il numero di droni (max 5), il numero di ostacoli (max 10) e un seed per la mappa. Es: .\build\DroneTest.exe mission 3 5 42
- **Energy Management**: Se la batteria scende sotto al BATTERY_LOW_THRESHOLD (50%), il drone ignorerà l'esplorazione e utilizzerà la rete neurale per tornare alla sua **Base di Ricarica** (punto di spawn originario). Lì taglierà la spinta e aspetterà di raggiungere il 95% di batteria prima di riprendere la missione.
- Verranno generati i file mission_telemetry.csv e mission_environment.csv.

---

## 📈 Visualizzazione e Strumentazione

Sono forniti diversi script Python per analizzare i risultati e visualizzare le simulazioni 3D. 

#### Andamento dell'Addestramento (Fitness & Tempi)
Per vedere quanto la rete neurale sta migliorando nel corso delle generazioni:
`powershell
python plot_fitness.py
`
Per visualizzare i tempi effettivi di calcolo per ogni singola generazione:
`powershell
python build\plot_times.py
`

#### Visualizzazione 3D: Training e Test
Per visualizzare in 3D le traiettorie della modalità di Test o dell'individuo migliore del Training:
`powershell
python build\visualize.py
`

#### Visualizzazione 3D: Missione Dinamica
Per visualizzare in tempo reale lo stato delle batterie, i target dinamici che cambiano nel tempo, e le basi di ricarica generate dalla modalità Mission:
`powershell
python build\visualize_mission.py
`
*(Nota: Nello script visualize_mission.py è presente una variabile SHOW_LIDAR. se impostata su True, verranno visualizzati i raggi LIDAR per capire esattamente cosa stanno "guardando" i droni).*

---
