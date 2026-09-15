import pandas as pd
import matplotlib.pyplot as plt
import os

FILENAME = "generation_times.csv"

def main():
    if not os.path.exists(FILENAME):
        print(f"Error: {FILENAME} non trovato. Assicurati di aver fatto girare l'addestramento.")
        return
        
    df = pd.read_csv(FILENAME)
    
    plt.figure(figsize=(10, 6))
    plt.plot(df['Generation'], df['TimeSeconds'], color='teal', linewidth=1.5, alpha=0.6, label='Tempo Effettivo')
    
    # Calcola media mobile per smussare il grafico
    if len(df) >= 50:
        ma = df['TimeSeconds'].rolling(window=50).mean()
        plt.plot(df['Generation'], ma, color='red', linestyle='-', linewidth=2, label='Media Mobile (50 Gen)')
        
    plt.title('Tempo di Computazione per Generazione')
    plt.xlabel('Generazione')
    plt.ylabel('Tempo (Secondi)')
    plt.grid(True, linestyle='--', alpha=0.5)
    plt.legend()
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    main()
