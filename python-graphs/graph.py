import pandas as pd
import matplotlib.pyplot as plt

dfPosition = pd.read_table('robotPosition', delim_whitespace=True, names=('Time','X','Y','Orientation'))

start = [dfPosition.X[0],dfPosition.Y[0]]
end = [dfPosition.X[dfPosition.X.count()-1],dfPosition.Y[dfPosition.Y.count()-1]]
print(start,end)

plt.plot(dfPosition['X'],dfPosition['Y'])
plt.plot(start[0],start[1],'o',color='g')
plt.plot(end[0],end[1],'o',color='r')

plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.grid(True)
plt.title('Robot Position')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.tight_layout()
plt.savefig('images/path.png')
plt.show()

plt.scatter(dfPosition['X'],dfPosition['Y'],s=1)

plt.plot(start[0],start[1],'o',color='g')
plt.plot(end[0],end[1],'o',color='r')
plt.tight_layout()
plt.show()

###############

dfBehavior = pd.read_table('behaviorOutput', delim_whitespace=True, names=('Time','B0','F0','B1','F1','B2','F2','B2','F2','B3','F3','B4','F4'))

fig =plt.subplots(2,2,sharex='col',sharey='row')

plt.subplot(2,2,1)
plt.plot(dfBehavior.Time, dfBehavior.F0, color='b',label='FlagB0')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(2,2,2)
plt.plot(dfBehavior.Time, dfBehavior.F1, color='g', label='FlagB1')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(2,2,3)
plt.plot(dfBehavior.Time, dfBehavior.F2, color='r', label='FlagB3')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(2,2,4)
plt.plot(dfBehavior.Time, dfBehavior.F3, color='c', label='FlagB4')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')


plt.tight_layout()
plt.show()


##################
##################
# BATERIA


dfBattery = pd.read_table('batteryOutput', delim_whitespace=True, names=('Time','battery','L0','L1','L2','L3','L4','L5','L6','L7', 'A2','A0','A1'))

#print(dfBattery.to_string())

plt.plot(dfBattery.Time, dfBattery.battery, color='r', label='Battery')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
#plt.ylim(0,1)
plt.xlabel('Timestep')
plt.ylabel('Battery')
plt.title('Nivel de Bateria')
plt.savefig('images/bateria.png')
plt.show()


######


posBat = pd.concat([dfPosition.X, dfPosition.Y, dfBattery.battery], axis=1, keys=['X','Y', 'battery'])
plt.scatter(posBat.X,posBat.Y,c=posBat.battery,s=100)
plt.colorbar()
plt.clim(0,1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Nivel de bateria vs Posicion')
plt.savefig('images/bateriaPos.png')
plt.show()


fig =plt.subplots(1,2,sharex='col',sharey='row')

plt.subplot(1,2,1)
plt.plot(dfBattery.Time, dfBattery.battery, color='r', label='Battery')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.title('Nivel de bateria')

plt.subplot(1,2,2)
plt.scatter(posBat.X,posBat.Y,c=posBat.battery,s=50)
plt.colorbar()
plt.title('Nivel de bateria vs Posicion')

plt.tight_layout()
plt.savefig('images/bateriaPosSubplot.png')
plt.show()

