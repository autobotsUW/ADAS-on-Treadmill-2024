import rtamt
import matplotlib.pyplot as plt
import numpy as np

# Création d'un moniteur STL
spec = rtamt.StlDiscreteTimeSpecification()

# Ajout de variables
spec.declare_var('x', 'float')

# Définition de la formule STL
spec.spec = 'G[0,50] (x >= -1 and x <= 1)'

# Compilation de la spécification
spec.parse()

# Génération de la trace de signal
n=1000
mu=0
sigma=0.1
# Création de la trace de signal avec vecteur de temps
Lt=[t for t in range(1000)]
# Lx=np.random.normal(mu, sigma, n)
Lx=[1.5*np.sin(2*np.pi*t/400) for t in Lt]
trace = {
    'time': Lt,
    'x': Lx
}

# Évaluation de la formule STL sur la trace de signal
robustness = spec.evaluate(trace)
# print(robustness)
# assert False

# Extraction des données de robustesse
times = [t for t, r in robustness]
robustness_values = [r for t, r in robustness]

# Tracé des signaux et de la robustesse
fig, ax1 = plt.subplots()

# Tracé de la valeur du signal x(t)
ax1.set_xlabel('Time')
ax1.set_ylabel('Signal x(t)', color='tab:blue')
ax1.plot(trace['time'], trace['x'], label='x(t)', color='tab:blue')
ax1.tick_params(axis='y', labelcolor='tab:blue')
ax1.set_ylim(-2, 2)
ax1.axhline(y=-1, color='gray', linestyle='--')
ax1.axhline(y=1, color='gray', linestyle='--')

# Tracé de la robustesse
ax2 = ax1.twinx()
ax2.set_ylabel('Robustness', color='tab:red')
ax2.plot(times, robustness_values, label='Robustness', color='tab:red')
ax2.tick_params(axis='y', labelcolor='tab:red')
ax2.set_ylim(-2, 2)

# Ajout d'une ligne horizontale à y=0 pour montrer la frontière de la satisfaction
ax2.axhline(y=0, color='gray', linestyle='--')


fig.tight_layout()
plt.title('Signal x(t) and STL Robustness')
plt.show()
