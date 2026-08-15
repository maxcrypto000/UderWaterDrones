cmake_minimum_required(VERSION 3.10)
project(UnderwaterDroneES C)

# Imposta lo standard C
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED True)

# Specifica il percorso dove hai estratto/compilato FMILibrary
# Assicurati di cambiare questo percorso con quello reale sul tuo computer
set(FMIL_DIR "${CMAKE_SOURCE_DIR}/lib/FMILibrary")

# Aggiungi la directory degli header (.h) della libreria
include_directories(${FMIL_DIR}/include)
include_directories(include)

# Aggiungi il file eseguibile
file(GLOB SOURCES "src/*.c")
add_executable(DroneTest ${SOURCES})

# Diciamo a CMake dove cercare i binari compilati
target_link_directories(DroneTest PRIVATE "${FMIL_DIR}/lib")

# Linkiamo la libreria per nome (troverà automaticamente libfmilib.a o fmilib.lib)
target_link_libraries(DroneTest PRIVATE fmilib_shared)