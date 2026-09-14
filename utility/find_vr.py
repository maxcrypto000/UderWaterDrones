import zipfile
import xml.etree.ElementTree as ET
import os

def extract_value_references(fmu_path, target_vars):
    if not os.path.exists(fmu_path):
        print(f"Errore: Il file '{fmu_path}' non esiste.")
        return

    try:
        # 1. Open the FMU as a ZIP archive without extracting it to disk
        with zipfile.ZipFile(fmu_path, 'r') as fmu_zip:
            # 2. Read the XML file directly
            with fmu_zip.open('modelDescription.xml') as xml_file:
                tree = ET.parse(xml_file)
                root = tree.getroot()

        print(f"Analisi del file: {fmu_path}\n")
        print(f"{'Variabile':<15} | {'Value Reference (VR)'}")
        print("-" * 40)

        # 3. Navigate the XML to find the <ModelVariables> tag
        found_variables = set()
        for scalar_var in root.findall('.//ScalarVariable'):
            name = scalar_var.get('name')
            vr = scalar_var.get('valueReference')

            if name in target_vars:
                print(f"{name:<15} | {vr}")
                found_variables.add(name)

        # Check if anything is missing
        missing_variables = set(target_vars) - found_variables
        if missing_variables:
            print("\nATTENZIONE - Le seguenti variabili non sono state trovate:")
            for missing in missing_variables:
                print(f"- {missing}")
            print("Sei sicuro di aver disabilitato l'Evaluate o che non siano state eliminate dal compilatore Modelica?")

    except Exception as e:
        print(f"Si è verificato un errore imprevisto: {e}")

if __name__ == "__main__":
    # --- CONFIGURATION ---
    # Insert the path of your FMU file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    fmu_file_path = os.path.join(script_dir, "..", "fmu", "UnderwaterDronePhysics.fmu")
    
    # Insert the exact names of the variables you want to find the VR for
    target_variables = [
        "ux", "uy", "uz",                # Input
        "x", "y", "z",                   # Position Output
        "vx", "vy", "vz",                # Velocity Output
        "start_x", "start_y", "start_z", # Parameters
        "battery"                        # Battery Output
    ]
    
    extract_value_references(fmu_file_path, target_variables)