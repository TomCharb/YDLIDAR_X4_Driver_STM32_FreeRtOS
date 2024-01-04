import serial
import matplotlib

matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import math

# Paramètres du port série
port = 'COM7'  # Remplace cela par ton port série (COM1, COM2, etc. sous Windows)
vitesse = 230400  # Vitesse en bauds (bits par seconde)

# Ouverture du port série
ser = serial.Serial(port, vitesse, timeout=1)

# Tableau pour stocker les valeurs reçues
valeurs = []

plt.ion()
fig = plt.figure()
ax = fig.add_subplot(111, polar=True)
line, = ax.plot([], [], marker='.', linestyle='')

plt.show()

while True:
    ligne = ser.readline().decode().strip()  # Lecture d'une ligne depuis le port série
    if ligne == '#':  # Si la ligne contient uniquement '#'
        while True:
            ligne = ser.readline().decode().strip()  # Lecture d'une ligne depuis le port série
            if ligne == '#':  # Si la ligne contient uniquement '#', on a atteint la fin des 360 valeurs
                angles = range(0, 360, 36)  # Liste des angles de 0 à 358 par pas de 2
                distances = valeurs  # Utilisation des valeurs lues comme distances
                ax.clear()  # Nettoyer le tracé précédent
                ax.set_theta_direction(-1)  # Sens des angles trigonométriques
                ax.set_theta_zero_location('N')  # Direction zéro (Nord)
                line, = ax.plot([math.radians(a) for a in angles], distances, marker='.', linestyle='')
                fig.canvas.draw()
                fig.canvas.flush_events()

                valeurs.clear()

            else:
                # Sinon, ajoute la valeur convertie en float au tableau
                valeur = float(ligne)
                valeurs.append(valeur)

# Fermeture du port série à la fin de l'utilisation
ser.close()

# Vérification du tableau de valeurs
print("Valeurs reçues :", valeurs)
print(len(valeurs))
