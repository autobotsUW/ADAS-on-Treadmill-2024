# import rtamt
# import matplotlib.pyplot as plt
# import numpy as np

# # Création d'un moniteur STL
# spec = rtamt.StlDiscreteTimeSpecification()

# # Ajout de variables
# spec.declare_var('x', 'float')

# # Définition de la formule STL
# spec.spec = 'G[0,50] (x >= -1 and x <= 1)'

# # Compilation de la spécification
# spec.parse()

# # Génération de la trace de signal
# n=1000
# mu=0
# sigma=0.1
# # Création de la trace de signal avec vecteur de temps
# Lt=[t for t in range(1000)]
# # Lx=np.random.normal(mu, sigma, n)
# Lx=[1.5*np.sin(2*np.pi*t/400) for t in Lt]
# trace = {
#     'time': Lt,
#     'x': Lx
# }

# # Évaluation de la formule STL sur la trace de signal
# robustness = spec.evaluate(trace)
# # print(robustness)
# # assert False

# # Extraction des données de robustesse
# times = [t for t, r in robustness]
# robustness_values = [r for t, r in robustness]

# # Tracé des signaux et de la robustesse
# fig, ax1 = plt.subplots()

# # Tracé de la valeur du signal x(t)
# ax1.set_xlabel('Time')
# ax1.set_ylabel('Signal x(t)', color='tab:blue')
# ax1.plot(trace['time'], trace['x'], label='x(t)', color='tab:blue')
# ax1.tick_params(axis='y', labelcolor='tab:blue')
# ax1.set_ylim(-2, 2)
# ax1.axhline(y=-1, color='gray', linestyle='--')
# ax1.axhline(y=1, color='gray', linestyle='--')

# # Tracé de la robustesse
# ax2 = ax1.twinx()
# ax2.set_ylabel('Robustness', color='tab:red')
# ax2.plot(times, robustness_values, label='Robustness', color='tab:red')
# ax2.tick_params(axis='y', labelcolor='tab:red')
# ax2.set_ylim(-2, 2)

# # Ajout d'une ligne horizontale à y=0 pour montrer la frontière de la satisfaction
# ax2.axhline(y=0, color='gray', linestyle='--')


# fig.tight_layout()
# plt.title('Signal x(t) and STL Robustness')
# plt.show()









# import matplotlib.pyplot as plt
# import numpy as np
# from itertools import count
# import time
# import rtamt

# # Création d'un moniteur STL
# spec = rtamt.StlDiscreteTimeSpecification()
# spec.declare_var('y', 'float')
# spec.spec = 'G[0,50] (y >= 0.3)'  # Exemple de formule STL à évaluer

# # Fonction pour simuler la récupération continue des données
# def get_data():
#     for i in count():
#         # Simuler une nouvelle donnée
#         yield i, np.random.random()  # Exemple avec un index i et une valeur aléatoire

# # Paramètres du tracé
# plt.ion()  # Mode interactif
# fig, ax = plt.subplots()
# line, = ax.plot([], [], lw=2)
# ax.set_ylim(0, 1)  # Ajuster la limite y selon vos besoins

# # Initialisation des listes pour les données
# xdata, ydata = [], []

# # Fonction de mise à jour du tracé
# def update_plot(new_x, new_y):
#     xdata.append(new_x)
#     ydata.append(new_y)
#     line.set_data(xdata, ydata)
#     ax.relim()
#     ax.autoscale_view()
#     fig.canvas.draw()
#     fig.canvas.flush_events()

# # Boucle pour la récupération et la mise à jour des données
# for x, y in get_data():
#     # Mise à jour du tracé avec les nouvelles données
#     update_plot(x, y)
    
#     # Évaluation de la spécification STL sur la nouvelle donnée
#     trace = {
#         'time': [x],
#         'y': [y]
#     }
        
#     robustness = spec.evaluate(trace)

    
#     # Affichage du résultat de l'évaluation STL
#     for t, r in robustness:
#         print(f"Time: {t}, Robustness: {r}")
    
#     time.sleep(0.1)  # Délai pour simuler l'arrivée des données en temps réel

# # Désactiver le mode interactif et afficher le graphique final
# plt.ioff()
# plt.show()










import sys
import rtamt

def monitor():
    a1 = [(0, 3), (3, 2)]
    b1 = [(0, 2), (2, 5), (4, 1), (7, -7)]

    a2 = [(5, 6), (6, -2), (8, 7), (11, -1)]
    b2 = [(10, 4)]

    a3 = [(13, -6), (15, 0)]
    b3 = [(15, 0)]

    # # stl
    spec = rtamt.StlDenseTimeSpecification()
    spec.name = 'STL dense-time specification'
    spec.declare_var('a', 'float')
    spec.spec = 'a>=2'
    try:
        spec.parse()
    except rtamt.RTAMTException as err:
        print('RTAMT Exception: {}'.format(err))
        sys.exit()

    rob = spec.update(['a', a1], ['b', b1])
    print('rob: ' + str(rob))

    rob = spec.update(['a', a2], ['b', b2])
    print('rob: ' + str(rob))

    rob = spec.update(['a', a3], ['b', b3])
    print('rob: ' + str(rob))

if __name__ == '__main__':
    monitor()
