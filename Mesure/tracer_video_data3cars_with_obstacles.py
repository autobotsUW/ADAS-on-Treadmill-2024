import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as patches

file_name='Mesure 2024-07-16 14_44.csv' 
file=open(file_name,'r')
lines=file.readlines()
file.close()


Lt=[]
Lx0=[]
Ly0=[]
Langle0=[]
Lwidth0=[]
Llength0=[]
Lx1=[]
Ly1=[]
Langle1=[]
Lwidth1=[]
Llength1=[]
Lx2=[]
Ly2=[]
Langle2=[]
Lwidth2=[]
Llength2=[]

Linputx0=[]
Linputy0=[]
Linputx1=[]
Linputy1=[]
Linputx2=[]
Linputy2=[]

Lobstacle1_x=[]
Lobstacle1_y=[]
Lobstacle1_radius=[]
Lobstacle2_x=[]
Lobstacle2_y=[]
Lobstacle2_radius=[]
Lobstacle3_x=[]
Lobstacle3_y=[]
Lobstacle3_radius=[]

treadmill=[]
lanes=[]


for line in lines[2:]:
    line=line.replace('\n','')
    group_line=line.split(';    ;')
    # print(group_line)
    
    Lt.append(float(group_line[0]))
    print(Lt[-1])

    car=group_line[1].split(';')
    car0=False
    car1=False
    car2=False
    for i in range(0,len(car),6):
        if car[i]=='0':
            Lx0.append(int(car[i+1]))
            Ly0.append(int(car[i+2]))
            Langle0.append(-int(car[i+3]))
            Lwidth0.append(int(car[i+4]))
            Llength0.append(int(car[i+5]))
            car0=True
        if car[i]=='1':
            Lx1.append(int(car[i+1]))
            Ly1.append(int(car[i+2]))
            Langle1.append(-int(car[i+3]))
            Lwidth1.append(int(car[i+4]))
            Llength1.append(int(car[i+5]))
            car1=True
        if car[i]=='2':
            Lx2.append(int(car[i+1]))
            Ly2.append(int(car[i+2]))
            Langle2.append(-int(car[i+3]))
            Lwidth2.append(int(car[i+4]))
            Llength2.append(int(car[i+5]))
            car2=True
    if car0==False:
        Lx0.append(None)
        Ly0.append(None)
        Langle0.append(None)
        Lwidth0.append(None)
        Llength0.append(None)
    if car1==False:
        Lx1.append(None)
        Ly1.append(None)
        Langle1.append(None)
        Lwidth1.append(None)
        Llength1.append(None)
    if car2==False:
        Lx2.append(None)
        Ly2.append(None)
        Langle2.append(None)
        Lwidth2.append(None)
        Llength2.append(None)

    input0=False
    input1=False
    input2=False
    input=group_line[2].split(';')
    for i in range(0,len(input),3):
        if input[i]=='0':
            Linputx0.append(int(input[i+1]))
            Linputy0.append(int(input[i+2]))
            input0=True
        if input[i]=='1':
            Linputx1.append(int(input[i+1]))
            Linputy1.append(int(input[i+2]))
            input1=True
        if input[i]=='2':
            Linputx2.append(int(input[i+1]))
            Linputy2.append(int(input[i+2]))
            input2=True
    if input0==False:
        Linputx0.append(None)
        Linputy0.append(None)
    if input1==False:
        Linputx1.append(None)
        Linputy1.append(None)
    if input2==False:
        Linputx2.append(None)
        Linputy2.append(None)

    data=group_line[4].split(';')
    if len(treadmill)==0:
        treadmill.append(int(data[0])*2)
        treadmill.append(int(data[1])*2)
        for i in range(2,len(data)):
            lanes.append(int(data[i]))
    elif len(data)-2==5:
        lanes=[]
        for i in range(2,len(data)):
            lanes.append(int(data[i]))
    if treadmill[0]<int(data[0])*2:
        treadmill[0]=int(data[0])*2
        if treadmill[1]<int(data[1])*2:
            treadmill[1]=int(data[1])*2

    if group_line[5]!='':
        obstacle=group_line[5].split(';')
        if len(obstacle)>=3:
            Lobstacle1_x.append(int(obstacle[0]))
            Lobstacle1_y.append(int(obstacle[1]))
            Lobstacle1_radius.append(int(obstacle[2]))
        else:
            Lobstacle1_x.append(-10)
            Lobstacle1_y.append(-10)
            Lobstacle1_radius.append(0)

        if len(obstacle)>=6:
            Lobstacle2_x.append(int(obstacle[3]))
            Lobstacle2_y.append(int(obstacle[4]))
            Lobstacle2_radius.append(int(obstacle[5]))
        else:
            Lobstacle2_x.append(-10)
            Lobstacle2_y.append(-10)
            Lobstacle2_radius.append(0)
        if len(obstacle)>=9:
            Lobstacle3_x.append(int(obstacle[6]))
            Lobstacle3_y.append(int(obstacle[7]))
            Lobstacle3_radius.append(int(obstacle[8]))
        else:
            Lobstacle3_x.append(-10)
            Lobstacle3_y.append(-10)
            Lobstacle3_radius.append(0)
    else:
        Lobstacle1_x.append(-10)
        Lobstacle1_y.append(-10)
        Lobstacle1_radius.append(0)
        Lobstacle2_x.append(-10)
        Lobstacle2_y.append(-10)
        Lobstacle2_radius.append(0)
        Lobstacle3_x.append(-10)
        Lobstacle3_y.append(-10)
        Lobstacle3_radius.append(0)

    


print('treadmill:',treadmill)
print('lanes:',lanes)
# Exemple de DataFrame nettoyé avec deux voitures et des obstacles dynamiques
# df_cleaned = pd.DataFrame({
#     'time': [0, 1, 2, 3, 4],
#     'car1_x': [0, 1, 2, 3, 4],
#     'car1_y': [0, 1, 0, -1, 0],
#     'car1_angle': [0, 10, 20, 30, 40],
#     'car1_width': [2, 2, 2, 2, 2],
#     'car1_length': [4, 4, 4, 4, 4],
#     'car2_x': [4, 3, 2, 1, 0],
#     'car2_y': [0, -1, 0, 1, 0],
#     'car2_angle': [0, -10, -20, -30, -40],
#     'car2_width': [2, 2, 2, 2, 2],
#     'car2_length': [4, 4, 4, 4, 4],
#     'car0_x': [2, 2.5, 3, 3.5, 4],
#     'car0_y': [2, 1.5, 1, 0.5, 0],
#     'car0_angle': [0, 15, 30, 45, 60],
#     'car0_width': [2, 2, 2, 2, 2],
#     'car0_length': [4, 4, 4, 4, 4],
#     # Obstacles dynamiques (x, y, rayon)
#     'obstacle1_x': [1, 1.5, 2, 2.5, 3],
#     'obstacle1_y': [3, 2.5, 2, 1.5, 1],
#     'obstacle1_radius': [1, 1, 1, 1, 1],
#     'obstacle2_x': [3, 2.5, 2, None, None],
#     'obstacle2_y': [-2, -1.5, -1, None, None],
#     'obstacle2_radius': [1, 1, 1, None, None],
# })
# print(len(Lt),len(Lx0),len(Ly0),len(Langle0),len(Lwidth0),len(Llength0),len(Lx1),len(Ly1),len(Langle1),len(Lwidth1),len(Llength1),len(Lx2),len(Ly2),len(Langle2),len(Lwidth2),len(Llength2),len(Linputx0),len(Linputy0),len(Linputx1),len(Linputy1),len(Linputx2),len(Linputy2),len(Lobstacle1_x),len(Lobstacle1_y),len(Lobstacle1_radius),len(Lobstacle2_x),len(Lobstacle2_y),len(Lobstacle2_radius),len(Lobstacle3_x),len(Lobstacle3_y),len(Lobstacle3_radius))
df_data=pd.DataFrame({
    'time': Lt,
    'car0_x': Lx0,
    'car0_y': Ly0,
    'car0_angle': Langle0,
    'car0_width': Lwidth0,
    'car0_length': Llength0,
    'car1_x': Lx1,
    'car1_y': Ly1,
    'car1_angle': Langle1,
    'car1_width': Lwidth1,
    'car1_length': Llength1,
    'car2_x': Lx2,
    'car2_y': Ly2,
    'car2_angle': Langle2,
    'car2_width': Lwidth2,
    'car2_length': Llength2,
    'input0_x': Linputx0,
    'input0_y': Linputy0,
    'input1_x': Linputx1,
    'input1_y': Linputy1,
    'input2_x': Linputx2,
    'input2_y': Linputy2,
    'obstacle1_x': Lobstacle1_x,
    'obstacle1_y': Lobstacle1_y,
    'obstacle1_radius': Lobstacle1_radius,
    'obstacle2_x': Lobstacle2_x,
    'obstacle2_y': Lobstacle2_y,
    'obstacle2_radius': Lobstacle2_radius,
    'obstacle3_x': Lobstacle3_x,
    'obstacle3_y': Lobstacle3_y,
    'obstacle3_radius': Lobstacle3_radius,
})

# Configuration de la figure
fig, ax = plt.subplots()
ax.set_xlim(0, treadmill[0]+50)
ax.set_ylim(0, treadmill[1])
ax.set_aspect('equal')
if len(lanes)==6:
    lanes.sort()
    # lanes.append((lanes[1]+lanes[2])/2)
    lanes.pop(1)
    # lanes.pop(1)
for lane in lanes:
    ax.plot([0,treadmill[0]],[lane,lane],color='black')

# Ajout des voitures comme des rectangles orientés

car0 = patches.Rectangle((0, 0), 0, 0, color='purple', label='Car 2')
car1 = patches.Rectangle((0, 0), 0, 0, color='red', label='Car 0')
car2 = patches.Rectangle((0, 0), 0, 0, color='blue', label='Car 1')
ax.add_patch(car1)
ax.add_patch(car2)
ax.add_patch(car0)

# Ajout des obstacles comme des cercles
obstacle1 = plt.Circle((0, 0), 0, color='green', label='Obstacle')
obstacle2 = plt.Circle((0, 0), 0, color='green')
obstacle3 = plt.Circle((0, 0), 0, color='green')
ax.add_patch(obstacle1)
ax.add_patch(obstacle2)
ax.add_patch(obstacle3)

# Fonction d'initialisation pour l'animation
def init():
    car1.set_xy((0, 0))
    car1.angle = 0
    car2.set_xy((0, 0))
    car2.angle = 0
    car0.set_xy((0, 0))
    car0.angle = 0
    obstacle1.center = (0, 0)
    obstacle2.center = (0, 0)
    obstacle3.center = (0, 0)
    return car1, car2, car0, obstacle1, obstacle2, obstacle3

# Fonction d'animation
def update(frame):
    t = df_data.iloc[frame]

    # Mise à jour de car0
    if not pd.isnull(t['car0_x']) and not pd.isnull(t['car0_y']) and not pd.isnull(t['car0_width']) and not pd.isnull(t['car0_length']):
        car0.set_xy((t['car0_x'] - t['car0_width'] / 2, t['car0_y'] - t['car0_length'] / 2))
        car0.angle = t['car0_angle']
        car0.set_width(t['car0_width'])
        car0.set_height(t['car0_length'])
    
    # Mise à jour de car1
    if not pd.isnull(t['car1_x']) and not pd.isnull(t['car1_y']) and not pd.isnull(t['car1_width']) and not pd.isnull(t['car1_length']):
        car1.set_xy((t['car1_x'] - t['car1_width'] / 2, t['car1_y'] - t['car1_length'] / 2))
        car1.angle = t['car1_angle']
        car1.set_width(t['car1_width'])
        car1.set_height(t['car1_length'])
    
    # Mise à jour de car2
    if not pd.isnull(t['car2_x']) and not pd.isnull(t['car2_y']) and not pd.isnull(t['car2_width']) and not pd.isnull(t['car2_length']):
        car2.set_xy((t['car2_x'] - t['car2_width'] / 2, t['car2_y'] - t['car2_length'] / 2))
        car2.angle = t['car2_angle']
        car2.set_width(t['car2_width'])
        car2.set_height(t['car2_length'])
        
    # Mise à jour des obstacles dynamiques
    if not pd.isnull(t['obstacle1_x']) and not pd.isnull(t['obstacle1_y']) and not pd.isnull(t['obstacle1_radius']):
        obstacle1.center = (t['obstacle1_x'], t['obstacle1_y'])
        obstacle1.set_radius(t['obstacle1_radius'])
    
    if not pd.isnull(t['obstacle2_x']) and not pd.isnull(t['obstacle2_y']) and not pd.isnull(t['obstacle2_radius']):
        obstacle2.center = (t['obstacle2_x'], t['obstacle2_y'])
        obstacle2.set_radius(t['obstacle2_radius'])
    
    if not pd.isnull(t['obstacle3_x']) and not pd.isnull(t['obstacle3_y']) and not pd.isnull(t['obstacle3_radius']):
        obstacle3.center = (t['obstacle3_x'], t['obstacle3_y'])
        obstacle3.set_radius(t['obstacle3_radius'])
    
    return car1, car2, car0, obstacle1, obstacle2, obstacle3

print('Start animation...')
# Création de l'animation
ani = FuncAnimation(fig, update, frames=len(df_data), init_func=init, blit=True)
print('Save animation...')
# Sauvegarder l'animation en vidéo
try:
    ani.save('{}_animation.mp4'.format(file_name), writer='ffmpeg', dpi=200, fps=30, codec='libx264', bitrate=5000)
except ValueError:
    print("MovieWriter ffmpeg unavailable; using Pillow instead.")
    ani.save('car_animation.gif', writer='pillow', fps=30)

# Ajout de la légende
ax.legend()
print('Show animation...')
# Afficher l'animation
plt.show(block=False)
plt.close()
print('End animation...')
