import matplotlib.pyplot as plt 


def tracer(path,aff=True):
    fichier=open(path,"r")
    lignes=fichier.readlines()
    fichier.close()

    Lt0=[]
    Lt1=[]
    Lx0=[]
    Lx0dim=[]
    Ly0=[]
    Ly0dim=[]
    Langle0=[]
    Lx1=[]
    Lx1dim=[]
    Ly1=[]
    Ly1dim=[]
    Langle1=[]

    Lt0input=[]
    Lt1input=[]
    Lx0input=[]
    Ly0input=[]
    Lx1input=[]
    Ly1input=[]

    Lt0command=[]
    Lt1command=[]
    Lspeed0=[]
    Lspeed1=[]
    Ldirection0=[]
    Ldirection1=[]

    LtxObstacles=[]
    LtyObstacles=[]
    LObstaclesx=[]
    LObstaclesy=[]    
    LdimxObstacles=[]      
    LdimyObstacles=[]

    for ligne in lignes[2:]:
        data=ligne.split("  ")
        t=float(data[0].replace(";",""))

        cars=data[2].split(";")
        cars.pop()
        cars.pop(0)
        Lxcars=[]
        for i in range(0,len(cars),6):
            if cars[i]=="0":
                Lt0.append(t)
                Lx0.append(float(cars[i+1]))
                Ly0.append(float(cars[i+2]))
                Langle0.append(float(cars[i+3]))
                Lx0dim.append(float(cars[i+4]))
                Ly0dim.append(float(cars[i+5]))

                Lxcars.append(float(cars[i+1]))
            elif cars[i]=="1":
                Lt1.append(t)
                Lx1.append(float(cars[i+1]))
                Ly1.append(float(cars[i+2]))
                Langle1.append(float(cars[i+3]))
                Lx1dim.append(float(cars[i+4]))
                Ly1dim.append(float(cars[i+5]))
                
                Lxcars.append(float(cars[i+1]))

        input=data[4].split(";")
        input.pop()
        input.pop(0)
        for i in range(0,len(input),3):
            if input[i]=="0":
                Lt0input.append(t)
                Lx0input.append(float(input[i+1]))
                Ly0input.append(float(input[i+2]))
            elif input[i]=="1":
                Lt1input.append(t)
                Lx1input.append(float(input[i+1]))
                Ly1input.append(float(input[i+2]))

        command=data[6].split(";")
        # print(command)
        command.pop()
        command.pop(0)
        for i in range(0,len(command),3):
            if command[i]=="0":
                Lt0command.append(t)
                Lspeed0.append(float(command[i+1]))
                Ldirection0.append(float(command[i+2]))
            elif command[i]=="1":
                Lt1command.append(t)
                Lspeed1.append(float(command[i+1]))
                Ldirection1.append(float(command[i+2]))
        
        obstacles=data[8].split(";")
        # print(command)
        obstacles.pop(0)
        for i in range(0,len(obstacles),3):
            LtxObstacles.append(t)
            LObstaclesx.append(float(obstacles[i]))
            LdimxObstacles.append(float(obstacles[i+2])/2)
            Ld=[float(obstacles[i])-xcar for xcar in Lxcars] 
            Ld.sort()
            if len(Ld)>0 and Ld[0]<float(obstacles[i+2]):
                LtyObstacles.append(t)
                LObstaclesy.append(float(obstacles[i+1]))
                LdimyObstacles.append(float(obstacles[i+2])/2)

    # fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(10, 15))
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 15))

    # Premier sous-graphe
    ax1.plot(Lt0, Lx0, '+r', label='car 0')
    ax1.plot(Lt1, Lx1, '+b', label='car 1')
    # ax1.errorbar(Lt0, Lx0, yerr=Lx0dim, color='r', fmt='o', capsize=0, label='car 0')
    # ax1.errorbar(Lt1, Lx1, yerr=Lx1dim, color='b', fmt='o', capsize=0, label='car 1')
    ax1.plot(Lt0input, Lx0input, 'k', label='car 0 input')
    ax1.plot(Lt1input, Lx1input, 'k', label='car 1 input')
    ax1.errorbar(LtxObstacles, LObstaclesx, yerr=LdimxObstacles, color='g', fmt='+', capsize=0, label='Obstacles')
    ax1.set_ylim(0, 716)
    ax1.legend()
    ax1.set_title('Graphique 1 : Position X')

    # Deuxième sous-graphe
    ax2.plot(Lt0, Ly0, '+r', label='car 0')
    ax2.plot(Lt1, Ly1, '+b', label='car 1')
    # ax2.errorbar(Lt0, Ly0, yerr=Ly0dim, color='r', fmt='o', capsize=0, label='car 0')
    # ax2.errorbar(Lt1, Ly1, yerr=Ly1dim, color='b', fmt='o', capsize=0, label='car 1')
    ax2.plot(Lt0input, Ly0input, 'k', label='car 0 input')
    ax2.plot(Lt1input, Ly1input, 'k', label='car 1 input')
    ax2.errorbar(LtyObstacles, LObstaclesy, yerr=LdimyObstacles, color='g', fmt='+', capsize=0, label='Obstacles')
    ax2.set_ylim(0, 400)
    ax2.legend()
    ax2.set_title('Graphique 2 : Position Y')

    # Troisième sous-graphe
    # ax3.plot(Lt0command, Lspeed0, '+r', label='car 0')
    # ax3.plot(Lt1command, Lspeed1, '+b', label='car 1')
    # ax3.legend()
    # ax3.set_title('Graphique 3 : Command Speed')

    # ax3.plot(Lt0,Langle0,'+r',label='car 0')
    # ax3.plot(Lt1,Langle1,'+b',label='car 1')
    # ax3.set_ylim(-20, 20)
    # ax3.legend()
    # ax3.set_title('Graphique 3 : Angle')

    
    # ax3.set_ylim(0, 400)
    # ax3.legend()
    # ax3.set_title('Graphique 3 : Obstacles')


    # Afficher les sous-graphiques
    plt.tight_layout()
    plt.savefig("{}.png".format(path[:-4]))
    plt.show(block=aff)
    plt.pause(1e-3)
    plt.close()

# tracer("Mesure 2024-06-27 09:26:07.csv")

import os

# Chemin du dossier où se trouve votre programme Python
dossier_courant = os.getcwd()

# Liste tous les fichiers dans ce dossier
fichiers_dans_dossier = os.listdir(dossier_courant)

# Affiche la liste des fichiers
print("Mesures tracee:")
for fichier in fichiers_dans_dossier:
    if fichier[-4:]=='.csv' and ("{}.png".format(fichier[:-4]) not in fichiers_dans_dossier):
        print(fichier)
        tracer(fichier,False)








