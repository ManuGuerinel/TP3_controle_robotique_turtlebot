"""Module pour analyser et visualiser des trajectoires de TurtleBot à partir de fichiers CSV.
Qui doivent être situés dans le répertoire 'trajectoire_save' et contenir..."""

import os
import csv
import numpy as np
import matplotlib.pyplot as plt


REPERTOIRE_DATA = "../../../trajectoire_save"
REPERTOIRE_PLOT = "../../../plots"


def list_csv_fichiers():
    fichiers = [f for f in os.listdir(REPERTOIRE_DATA) if f.endswith(".csv")]
    return fichiers


def charger_csv(chemin_fichier):
    data = np.genfromtxt(chemin_fichier, delimiter=',', skip_header=1)
    # if data.ndim == 1:
    #     data = data.reshape(1, -1)

    time = data[:, 0]
    positions = data[:, 1:]
    return time, positions


def analyze_trajectory(time, positions):
    duration = time[-1]

    min_pos = np.min(positions, axis=0)
    max_pos = np.max(positions, axis=0)

    dt = np.diff(time)
    dpos = np.diff(positions, axis=0)
    velocities = np.linalg.norm(dpos, axis=1) / dt
    mean_velocity = np.mean(velocities)

    return duration, min_pos, max_pos, mean_velocity


def plot_trajectoire(time, positions, filename):
    os.makedirs(REPERTOIRE_PLOT, exist_ok=True)

    plt.figure(figsize=(10, 6))

    for i in range(positions.shape[1]):
        plt.plot(time, positions[:, i], label=f"Axe {i+1}")

    plt.xlabel("Temps (s)")
    plt.ylabel("Position")
    plt.title(f"Trajectoire : {filename}")
    plt.legend()
    plt.grid(True)

    chemin_sauvegarde = os.path.join(REPERTOIRE_PLOT, filename.replace(".csv", ".png"))
    plt.savefig(chemin_sauvegarde)
    plt.close()

    print(f"Graphique sauvegardé : {chemin_sauvegarde}")

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
        time, positions = charger_csv(filepath)

        duration, min_pos, max_pos, mean_velocity = analyze_trajectory(time, positions)

        print("\n--- Analyse de la trajectoire ---")
        print(f"Durée : {duration:.2f} s")
        for i in range(len(min_pos)):
            print(f"Axe {i+1} : min = {min_pos[i]:.3f}, max = {max_pos[i]:.3f}")
        print(f"Vitesse moyenne : {mean_velocity:.3f} unités/s")

        plot_trajectoire(time, positions, filename)


if __name__ == "__main__":
    main()
