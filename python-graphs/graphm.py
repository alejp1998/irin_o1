import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

dfSensorLuz = pd.read_table('sensorLuzOutput', delim_whitespace=True, names=('Time','totalLight','totalBlueLight','totalRedLight'))
dfPositionLuz = pd.read_table('robotPosition', delim_whitespace=True, names=('Time','X','Y','Orientation'))

posLightY = pd.DataFrame(columns=[])
posLightB = pd.DataFrame(columns=[])
posLightR = pd.DataFrame(columns=[])

def concat(array,columna, nombre):
	return pd.concat([dfPositionLuz.X, dfPositionLuz.Y, columna], axis=1, keys=['X','Y', nombre])

def eliminar(array,nombre):
	return array.sort_values(nombre).drop_duplicates(subset=['X', 'Y'], keep='last').reset_index(drop=True)

def norm(array,nombre):
	columna = array[nombre]
	array.drop([nombre], axis=1, inplace=True)
	array[nombre] = columna/columna.max()
	return array

def foo(array,columna,nombre):
	return norm(eliminar(concat(array,columna,nombre),nombre),nombre)

posLightY = foo(posLightY,dfSensorLuz.totalLight,'totalLight')
posLightB = foo(posLightB,dfSensorLuz.totalBlueLight,'totalBlueLight')
posLightR = foo(posLightR,dfSensorLuz.totalRedLight,'totalRedLight')


print(posLightY.totalLight.max())


##################
def scatter(array,columna,titulo,s):
	plt.scatter(array.X, array.Y,c=columna,s=s)
	plt.colorbar()
	plt.clim(0,1)
	plt.xlabel('X')
	plt.ylabel('Y')
	plt.title(titulo)
	plt.savefig('images/'+titulo+str(s)+'.png')
	plt.show()

# MAPAS DE CALOR LUCES
scatter(posLightY,posLightY.totalLight, 'Luz Amarilla',100)
scatter(posLightB,posLightB.totalBlueLight, 'Luz Azul',100)
scatter(posLightR,posLightR.totalRedLight, 'Luz Rojo',100)

## SUBPLOTS MAPAS DE CALOR LUCES
fig = plt.subplots(1,3,sharex='col',sharey='row')

plt.subplot(1,3,1)
plt.scatter(posLightY.X, posLightY.Y,c=posLightY.totalLight,s=100)
plt.colorbar()
plt.clim(0,1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Luz Amarilla')

plt.subplot(1,3,2)
plt.scatter(posLightB.X, posLightB.Y,c=posLightB.totalBlueLight,s=100)
plt.colorbar()
plt.clim(0,1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Luz Azul')

plt.subplot(1,3,3)
plt.scatter(posLightR.X, posLightR.Y,c=posLightR.totalRedLight,s=100)
plt.colorbar()
plt.clim(0,1)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Luz Rojo')

#plt.tight_layout()
plt.savefig('images/lucesSubplot.png')
plt.show()




#ZONAS MAS TRANSITADAS RESOLUCION 15X15 / 50X50
plt.hist2d(posLightY.X, posLightY.Y, bins=(15, 15), cmap=plt.cm.jet)
plt.title('Zonas mas transitadas - resolucion:15x15')
plt.xlabel('X')
plt.ylabel('Y')
plt.colorbar()
plt.savefig('images/zonas15.png')
plt.show()

plt.hist2d(posLightY.X, posLightY.Y, bins=(50, 50), cmap=plt.cm.jet)
plt.title('Zonas mas transitadas - resolucion:50x50')
plt.xlabel('X')
plt.ylabel('Y')
plt.colorbar()
plt.savefig('images/zonas50.png')
plt.show()

################
# KDEPLOT

#def kdeplot(array,columna,titulo):
sns.kdeplot(dfPositionLuz.X,dfPositionLuz.Y,cmap=plt.cm.jet)
plt.savefig('images/kdeplot.png')
plt.show()