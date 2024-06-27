import matplotlib.pyplot as plt 


fichier=open("Obstacles1ms.csv","r")
lignes=fichier.readlines()
fichier.close()

Lt=[]
Lx=[]
Ly=[]
Lx_input=[]
Ly_input=[]
for ligne in lignes:
    data=ligne.split(";")
    Lt.append(float(data[0]))
    Lx.append(float(data[1]))
    Ly.append(float(data[2]))
    Lx_input.append(float(data[3]))
    Ly_input.append(float(data[4]))


i=0
while i<len(Lt) and abs(Lx[i]-Lx[0])<2 and abs(Ly[i]-Ly[0])<2:
    i+=1
Lt2=[j-Lt[i] for j in Lt]

stanleyLt=Lt2
stanleyLx=Lx
stanleyLy=Ly

plt.plot(stanleyLt,stanleyLx,'r',label="Stanley")
plt.plot(stanleyLt,Lx_input,'k',label="Input")
plt.title("Comparaison des trajectoires en x à 0.60 m/s")
plt.xlabel("Temps (en s)")
plt.ylabel("x (en pixels)")
plt.ylim(0,716)
plt.savefig("Obstacles_x.png")
plt.legend()
plt.show()

plt.plot(stanleyLt,stanleyLy,'r',label="Car")
plt.plot(stanleyLt,Ly_input,'k',label="Input")
plt.title("Trajectoire avec obstacles en y à 1 m/s")
plt.xlabel("Temps (en s)")
plt.ylabel("y (en pixels)")
plt.ylim(0,400)
plt.legend()
plt.savefig("Obstacles_y.png")
plt.show()




fichier=open("Stanley2.csv","r")
lignes=fichier.readlines()
fichier.close()

Lt=[]
Lx=[]
Ly=[]
for ligne in lignes:
    data=ligne.split(";")
    Lt.append(float(data[0]))
    Lx.append(float(data[1]))
    Ly.append(float(data[2]))


i=0
while i<len(Lt) and abs(Lx[i]-Lx[0])<2 and abs(Ly[i]-Ly[0])<2:
    i+=1
Lt2=[j-Lt[i] for j in Lt]

stanleyLt=Lt2
stanleyLx=Lx
stanleyLy=Ly


fichier=open("PID.csv","r")
lignes=fichier.readlines()
fichier.close()

Lt=[]
Lx=[]
Ly=[]
for ligne in lignes:
    data=ligne.split(";")
    Lt.append(float(data[0]))
    Lx.append(float(data[1]))
    Ly.append(float(data[2]))

i=0
while i<len(Lt) and abs(Lx[i]-Lx[0])<2 and abs(Ly[i]-Ly[0])<2:
    i+=1
Lt2=[j-Lt[i] for j in Lt]

Stanley1msLt=Lt2
Stanley1msLx=Lx
Stanley1msLy=Ly

if Stanley1msLt[0]<stanleyLt[0]:
    while Stanley1msLt[0]<stanleyLt[0]:
        Stanley1msLt.pop(0)
        Stanley1msLx.pop(0)
        Stanley1msLy.pop(0)
else:
    while stanleyLt[0]<Stanley1msLt[0]:
        stanleyLt.pop(0)
        stanleyLx.pop(0)
        stanleyLy.pop(0)

if Stanley1msLt[-1]<stanleyLt[-1]:
    while Stanley1msLt[-1]<stanleyLt[-1]:
        stanleyLt.pop(-1)
        stanleyLx.pop(-1)
        stanleyLy.pop(-1)
else:
    while stanleyLt[-1]<Stanley1msLt[-1]:
        Stanley1msLt.pop(-1)
        Stanley1msLx.pop(-1)
        Stanley1msLy.pop(-1)

Lx_input=[300 for i in range(len(stanleyLt))]
Ly_input=[200 for i in range(len(stanleyLt))]

plt.plot(stanleyLt,stanleyLx,'r',label="Stanley")
plt.plot(Stanley1msLt,Stanley1msLx,'b',label="PID")
plt.plot(stanleyLt,Lx_input,'k',label="Input")
plt.legend()
plt.title("Comparaison des trajectoires en x à 0.60 m/s")
plt.xlabel("Temps (en s)")
plt.ylabel("x (en pixels)")
plt.savefig("Stanley-PID_x.png")
plt.show()

plt.plot(stanleyLt,stanleyLy,'r',label="Stanley")
plt.plot(Stanley1msLt,Stanley1msLy,'b',label="PID")
plt.plot(stanleyLt,Ly_input,'k',label="Input")
plt.legend()
plt.title("Comparaison des trajectoires en y à 0.60 m/s")
plt.xlabel("Temps (en s)")
plt.ylabel("y (en pixels)")
plt.savefig("Stanley-PID_y.png")
plt.show()


plt.plot(stanleyLx,stanleyLy,'r',label="Stanley")
plt.plot(Stanley1msLx,Stanley1msLy,'b',label="PID")
plt.plot(Lx_input,Ly_input,'ko',label="Input")
plt.legend()
plt.title("Comparaison des trajectoires en xy à 0.60 m/s")
plt.xlabel("x (en pixels)")
plt.ylabel("y (en pixels)")
plt.savefig("Stanley-PID_xy.png")
plt.show()