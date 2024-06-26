import matplotlib.pyplot as plt 


fichier=open("2car (3).csv","r")
lignes=fichier.readlines()
fichier.close()

Lt0=[]
Lt1=[]
Lx0=[]
Ly0=[]
Langle0=[]
Lx1=[]
Ly1=[]
Langle1=[]

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
for ligne in lignes[2:]:
    data=ligne.split("  ")
    t=float(data[0].replace(";",""))

    cars=data[2].split(";")
    cars.pop()
    cars.pop(0)
    for i in range(0,len(cars),6):
        if cars[i]=="0":
            Lt0.append(t)
            Lx0.append(float(cars[i+1]))
            Ly0.append(float(cars[i+2]))
            Langle0.append(float(cars[i+3]))
        elif cars[i]=="1":
            Lt1.append(t)
            Lx1.append(float(cars[i+1]))
            Ly1.append(float(cars[i+2]))
            Langle1.append(float(cars[i+3]))

    input=data[4].split(";")
    input.pop()
    input.pop(0)
    for i in range(0,len(input),3):
        if input[i]=="0":
            Lx0input.append(float(input[i+1]))
            Ly0input.append(float(input[i+2]))
        elif input[i]=="1":
            Lx1input.append(float(input[i+1]))
            Ly1input.append(float(input[i+2]))

    command=data[6].split(";")
    print(command)
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

fig, (ax1, ax2,ax3) = plt.subplots(3, 1, figsize=(10, 15))

# Premier sous-graphe
ax1.plot(Lt0, Lx0, '+r', label='car 0')
ax1.plot(Lt1, Lx1, '+b', label='car 1')
ax1.plot(Lt0, Lx0input, 'r', label='car 0 input')
ax1.plot(Lt1, Lx1input, 'b', label='car 1 input')
ax1.legend()
ax1.set_title('Graphique 1 : Position X')

# Deuxième sous-graphe
ax2.plot(Lt0, Ly0, '+r', label='car 0')
ax2.plot(Lt1, Ly1, '+b', label='car 1')
ax2.plot(Lt0, Ly0input, 'r', label='car 0 input')
ax2.plot(Lt1, Ly1input, 'b', label='car 1 input')
ax2.legend()
ax2.set_title('Graphique 2 : Position Y')

# Troisième sous-graphe
# ax3.plot(Lt0command, Lspeed0, '+r', label='car 0')
# ax3.plot(Lt1command, Lspeed1, '+b', label='car 1')
# ax3.legend()
# ax3.set_title('Graphique 3 : Command Speed')

ax3.plot(Lt0,Langle0,'+r',label='car 0')
ax3.plot(Lt1,Langle1,'+b',label='car 1')
ax3.legend()
ax3.set_title('Graphique 3 : Angle')


# Afficher les sous-graphiques
plt.tight_layout()
plt.savefig("2car-3.png")
plt.show()






