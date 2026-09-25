import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

def find_latest_log():
    # Szuka najpierw w folderze ../logs, a jeśli nie ma, w bieżącym
    logs_dir = Path("../logs")
    if not logs_dir.exists():
        logs_dir = Path("logs")
    
    if logs_dir.exists():
        csv_files = list(logs_dir.glob("*.csv"))
        if csv_files:
            return str(max(csv_files, key=os.path.getmtime))
    
    current_dir = Path(".")
    csv_files = list(current_dir.glob("*.csv"))
    if csv_files:
        return str(max(csv_files, key=os.path.getmtime))
        
    return "log_simulation.csv"

def plot_log(filename):
    if not os.path.exists(filename):
        print(f"Błąd: Plik '{filename}' nie istnieje.")
        return

    print(f"Otwieranie pliku logu: {filename}")
    try:
        df = pd.read_csv(filename)
    except Exception as e:
        print(f"Nie udało się wczytać pliku {filename}: {e}")
        return

    # Ustalenie kolumny czasu
    time_col = "Time [s]" if "Time [s]" in df.columns else df.columns[0]

    # Ustawienie ciemnego motywu wykresów (4 podwykresy)
    plt.style.use('dark_background')
    fig, axes = plt.subplots(4, 1, figsize=(12, 13), sharex=True)
    
    # 1. Wykres pozycji (X, Y, Z) - zadana (SP) oraz aktualna (PV)
    # Oś X
    if 'X_pos_series_1' in df.columns:
        axes[0].plot(df[time_col], df['X_pos_series_1'], label='X Zadana (SP)', color="#291af0", linestyle='--')
    if 'X_pos_series_2' in df.columns:
        axes[0].plot(df[time_col], df['X_pos_series_2'], label='X Aktualna (PV)', color='#64c8ff')
        
    # Oś Y
    if 'Y_pos_series_1' in df.columns:
        axes[0].plot(df[time_col], df['Y_pos_series_1'], label='Y Zadana (SP)', color="#C71EAE", linestyle='--')
    if 'Y_pos_series_2' in df.columns:
        axes[0].plot(df[time_col], df['Y_pos_series_2'], label='Y Aktualna (PV)', color='#c864ff')
        
    # Oś Z
    if 'Z_pos_series_1' in df.columns:
        axes[0].plot(df[time_col], df['Z_pos_series_1'], label='Z Zadana (SP)', color="#944403", linestyle='--')
    if 'Z_pos_series_2' in df.columns:
        axes[0].plot(df[time_col], df['Z_pos_series_2'], label='Z Aktualna (PV)', color='#ffb450')
        
    axes[0].set_title('Pozycje drona w czasie: Zadane (SP) vs Aktualne (PV)', fontsize=12, color='#e0e6ed')
    axes[0].set_ylabel('Pozycja [m]')
    axes[0].legend(loc='upper right', fontsize=8, ncol=3)
    axes[0].grid(True, color='#253045', alpha=0.5)

    # 2. Wykres uchybów na osobnym podwykresie
    err_cols = [col for col in df.columns if col.startswith('err_')]
    if err_cols:
        colors = ['#ff7777', '#77ff77', '#7777ff', '#ffff77']
        for i, col in enumerate(err_cols):
            err_name = col.replace('err_', '').upper()
            c = colors[i % len(colors)]
            axes[1].plot(df[time_col], df[col], label=f'Uchyb {err_name}', color=c)
        axes[1].legend(loc='upper right', fontsize=8, ncol=3)
    else:
        axes[1].text(0.5, 0.5, 'Brak danych uchybów w pliku logu', horizontalalignment='center',
                     verticalalignment='center', transform=axes[1].transAxes, color='#888888')
        
    axes[1].set_title('Uchyby regulacji', fontsize=12, color='#e0e6ed')
    axes[1].set_ylabel('Uchyb [m]')
    axes[1].grid(True, color='#253045', alpha=0.5)

    # 3. Wykres prędkości (Velocities 1, 2, 3)
    if 'Velocities_series_1' in df.columns:
        axes[2].plot(df[time_col], df['Velocities_series_1'], label='Prędkość 1 (Vx)', color='#ffaa55')
    if 'Velocities_series_2' in df.columns:
        axes[2].plot(df[time_col], df['Velocities_series_2'], label='Prędkość 2 (Vy)', color='#55ffaa')
    if 'Velocities_series_3' in df.columns:
        axes[2].plot(df[time_col], df['Velocities_series_3'], label='Prędkość 3 (Vz)', color='#aaff55')
        
    axes[2].set_title('Prędkości drona', fontsize=12, color='#e0e6ed')
    axes[2].set_ylabel('Prędkość [m/s]')
    axes[2].legend(loc='upper right', fontsize=9)
    axes[2].grid(True, color='#253045', alpha=0.5)

    # 4. Wykres kątów (Roll / Pitch)
    if 'Roll_Pitch_series_1' in df.columns:
        axes[3].plot(df[time_col], df['Roll_Pitch_series_1'], label='Roll/Pitch Seria 1', color='#ff55ff')
    if 'Roll_Pitch_series_2' in df.columns:
        axes[3].plot(df[time_col], df['Roll_Pitch_series_2'], label='Roll/Pitch Seria 2', color='#ffff55', linestyle='--')
        
    axes[3].set_title('Kąty (Roll / Pitch)', fontsize=12, color='#e0e6ed')
    axes[3].set_ylabel('Wartość kąta')
    axes[3].legend(loc='upper right', fontsize=9)
    axes[3].set_xlabel('Czas [s]')
    axes[3].grid(True, color='#253045', alpha=0.5)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_to_plot = sys.argv[1]
    else:
        file_to_plot = find_latest_log()
    
    plot_log(file_to_plot)