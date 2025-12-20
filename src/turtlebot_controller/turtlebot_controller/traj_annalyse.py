"""Module pour analyser et visualiser des trajectoires de TurtleBot à partir de fichiers CSV.
Qui doivent être situés dans le répertoire 'trajectoire_save' et contenir..."""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt


REPERTOIRE_DATA = "../../../trajectoire_save"
REPERTOIRE_PLOT = "../../../plots"

### =================[ Fonctions utilitaires ]==================

def list_csv_fichiers():
    fichiers = [f for f in os.listdir(REPERTOIRE_DATA) if f.endswith(".csv")]
    return fichiers


def charger_csv(chemin_fichier):
    data = np.genfromtxt(chemin_fichier, delimiter=',', skip_header=1)
    # if data.ndim == 1:
    #     data = data.reshape(1, -1)

    nb_pos = int(data[0,0])-1
    nb_vel = int(data[0,3])-1
    time_pos = data[1:nb_pos, 0]
    time_vel = data[1:nb_vel, 3]
    positions = data[1:nb_pos, 1:3]
    vitesses = data[1:nb_vel, 4:6]
    return time_pos, positions, time_vel, vitesses

def min_liste(liste):
    "returne la valeur minimale d'une liste et son indice"
    min_val = liste[0]
    min_idx = 0 #indice de la valeur minimale
    for i, val in enumerate(liste):
        if val < min_val:
            min_val = val
            min_idx = i
    return {'idx': min_idx, 'val': min_val}


def max_liste(liste):
    "returne la valeur maximale d'une liste et son indice"
    max_val = liste[0]
    max_idx = 0 #indice de la valeur maximale
    for i, val in enumerate(liste):
        if val > max_val:
            max_val = val
            max_idx = i
    return {'idx': max_idx, 'val': max_val}

def analyze_trajectory(time_pos, time_vel, positions, vitesses):
    "retourn un tableau de l'annalyse des trajectoire"
    duration = {
        'pos' : time_pos[-1],
        'vel' : time_vel[-1]
    }

    min_pos_left = min_liste(positions[:,0])
    min_pos_right = min_liste(positions[:,1])
    max_pos_left = max_liste(positions[:,0])
    max_pos_right = max_liste(positions[:,1])

    min_vel_lin = min_liste(vitesses[:,0])
    min_vel_ang = min_liste(vitesses[:,1])
    max_vel_lin = max_liste(vitesses[:,0])
    max_vel_ang = max_liste(vitesses[:,1])

    mean_vel_lin = np.mean(vitesses[:,0])
    mean_vel_ang = np.mean(vitesses[:,1])

    annalyse = [
        [min_pos_left, max_pos_left],
        [min_pos_right, max_pos_right],
        [min_vel_lin, max_vel_lin],
        [min_vel_ang, max_vel_ang],
        [mean_vel_lin, mean_vel_ang]
    ]
    return duration, annalyse

def plot_trajectoire(time_pos, time_vel, positions, vitesses, filename):
    os.makedirs(REPERTOIRE_PLOT, exist_ok=True)

    fig, ax1 = plt.subplots(figsize=(10, 6))

    # Positions (echelle a gauche)
    ax1.plot(time_pos, positions[:, 0], label="Left Wheel Position", color='blue')
    ax1.plot(time_pos, positions[:, 1], label="Right Wheel Position", color='green')
    ax1.set_xlabel("Temps (s)")
    ax1.set_ylabel("Position", color='blue')
    ax1.tick_params(axis='y', labelcolor='blue')

    # Vitesses (echelle a droite)
    ax2 = ax1.twinx()
    ax2.plot(time_vel, vitesses[:, 0], label="linear velocity", color='red')
    ax2.plot(time_vel, vitesses[:, 1], label="angular Velocity", color='orange')
    ax2.set_ylabel("Vitesse", color='red')
    ax2.tick_params(axis='y', labelcolor='red')

    # calcul min, max, mean des vitesses
    _, annalyse = analyze_trajectory(time_pos, time_vel, positions, vitesses)
    mean_vel_lin = annalyse[4][0]
    mean_vel_ang = annalyse[4][1]

    # Plot ligne horizontal pour les vitesses moyennes
    ax2.axhline(mean_vel_lin, color='red', linestyle='--', linewidth=2, label='Mean Linear Velocity')
    ax2.axhline(mean_vel_ang, color='orange', linestyle='--', linewidth=2, label='Mean Angular Velocity')

    # For positions, plot crosses for min and max
    min_pos_left = annalyse[0][0]
    max_pos_left = annalyse[0][1]
    min_pos_right = annalyse[1][0]
    max_pos_right = annalyse[1][1]

    ax1.scatter(time_pos[min_pos_left['idx']], min_pos_left['val'], color='blue', marker='x', s=100, label='Min/Max Left Position')
    ax1.scatter(time_pos[max_pos_left['idx']], max_pos_left['val'], color='blue', marker='x', s=100)
    ax1.scatter(time_pos[min_pos_right['idx']], min_pos_right['val'], color='green', marker='x', s=100, label='Min/Max Right Position')
    ax1.scatter(time_pos[max_pos_right['idx']], max_pos_right['val'], color='green', marker='x', s=100)

    plt.title(f"Trajectoire : {filename}")
    # Combine les legendes
    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='upper center', bbox_to_anchor=(0.5, -0.15), ncol=2)
    plt.grid(True)

    plt.tight_layout()
    chemin_sauvegarde = os.path.join(REPERTOIRE_PLOT, filename.replace(".csv", ".png"))
    plt.savefig(chemin_sauvegarde)
    plt.close()

    print(f"Graphique sauvegardé : {chemin_sauvegarde}")

### =================[ Main ]==================

def main():
    while True:
        fichiers = list_csv_fichiers()

        if not fichiers:
            print("Aucun fichier CSV trouvé.")
            return

        print("\nFichiers disponibles :")
        for i, f in enumerate(fichiers):
            print(f"[{i}] {f}")

        index = input("\nChoisir un fichier (index) ou 'q' pour quitter : ")

        if index.lower() == 'q':
            print("Fin du programme.")
            break

        try:
            idx = int(index)
            filename = fichiers[idx]
        except (ValueError, IndexError):
            print("Choix invalide.")
            continue

        filepath = os.path.join(REPERTOIRE_DATA, filename)
        time_pos, positions, time_vel, vitesses = charger_csv(filepath)

        duration, annalyse_tab = analyze_trajectory(time_pos, time_vel , positions, vitesses)

        print("\n--- Analyse de la trajectoire ---")
        print(f"Durée : {duration['pos']:.2f} s")
        data_annalyse = ["position  left", "position right", "vitesse lineaire", "vitesse angulaire", "vitesse moyenne"]
        for i in range(len(annalyse_tab)-1):
            print(f"{data_annalyse[i]} : min = {annalyse_tab[i][0]['val']:.3f}, max = {annalyse_tab[i][1]['val']:.3f}")
        print(f"{data_annalyse[4]} : left = {annalyse_tab[4][0]:.3f}, right = {annalyse_tab[4][1]:.3f}")
        plot_trajectoire(time_pos, time_vel, positions, vitesses, filename)

if __name__ == "__main__":
    main()
