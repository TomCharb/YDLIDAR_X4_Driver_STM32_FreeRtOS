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
ax = fig.add_subplot(111)
line, = ax.plot([-2500, 2500], [-2500, 2500], marker='.', linestyle='')

plt.show()

while True:
    ligne = ser.readline().decode().strip()  # Lecture d'une ligne depuis le port série
    if ligne == '#':  # Si la ligne contient uniquement '#'
        while True:
            ligne = ser.readline().decode().strip()  # Lecture d'une ligne depuis le port série
            if ligne == '#':  # Si la ligne contient uniquement '#', on a atteint la fin des 360 valeurs
                # Coordonnées du centre
                centre = (0, 0)
                # Calcul des coordonnées des points en fonction des distances
                points_x = []
                points_y = []
                for degre, distance in enumerate(valeurs):
                    angle = math.radians(2 * degre)
                    x = centre[0] + distance * math.cos(-angle)
                    y = centre[1] + distance * math.sin(-angle)
                    points_x.append(x)
                    points_y.append(y)
                line.set_xdata(points_x)
                line.set_ydata(points_y)
                fig.canvas.draw()
                fig.canvas.flush_events()
                print(valeurs)
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
