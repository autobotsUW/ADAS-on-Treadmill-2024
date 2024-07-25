import matplotlib.pyplot as plt
import numpy as np
from itertools import count
import time

# Fonction pour simuler la récupération continue des données
def get_data():
    for i in count():
        # Simuler une nouvelle donnée
        yield i, np.random.random()  # Exemple avec un index i et une valeur aléatoire

# Paramètres du tracé
plt.ion()  # Mode interactif
fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2)
ax.set_ylim(0, 1)  # Ajuster la limite y selon vos besoins

# Initialisation des listes pour les données
xdata, ydata = [], []

# Fonction de mise à jour du tracé
def update_plot(new_x, new_y):
    xdata.append(new_x)
    ydata.append(new_y)
    line.set_data(xdata, ydata)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()

print('For')
# Boucle pour la récupération et la mise à jour des données
for x, y in get_data():
    update_plot(x, y)
    time.sleep(0.1)  # Délai pour simuler l'arrivée des données en temps réel

print('Show')
plt.ioff()
plt.show()
print('Fin')
