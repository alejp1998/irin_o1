import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


dfBehavior = pd.read_table("behaviorOutput", delim_whitespace=True, names=('Time','B0','F0','B1','F1','B2','F2','B3','F3','B4','F4'))

# DENTRO DEL COORDINATOR()
# FILE* fileBehaviorOutput = fopen("outputFiles/behaviorOutput", "a");
# fprintf(fileBehaviorOutput,"%2.4f ", m_fTime);
# for(int i=0; i < BEHAVIORS; i++){
# 	//printf("\n%d\n",i);
# 	fprintf(fileBehaviorOutput,"%d %2.4f ", i, m_fActivationTable[i][2]);
# }
# fprintf(fileBehaviorOutput,"\n");		
# fclose(fileBehaviorOutput);

fig =plt.subplots(5,1,sharex='col',sharey='row')
plt.tight_layout()

plt.subplot(5,1,1)
plt.plot(dfBehavior.Time, dfBehavior.F0, color='b',label='FlagAvoid')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')
#plt.legend(bbox_to_anchor=(1,1), borderaxespad=0)

plt.subplot(5,1,2)
plt.plot(dfBehavior.Time, dfBehavior.F1, color='g', label='FlagGoLoad')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(5,1,3)
plt.plot(dfBehavior.Time, dfBehavior.F2, color='r', label='FlagForage')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(5,1,4)
plt.plot(dfBehavior.Time, dfBehavior.F3, color='c', label='FlagNavigate')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(5,1,5)
plt.plot(dfBehavior.Time, dfBehavior.F4, color='m', label='FlagGo4Walle')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.savefig('images/behaviorSubplot.png')
plt.show()

####################
# SIN NAVIGATE

fig =plt.subplots(4,1,sharex='col',sharey='row')
plt.tight_layout()

plt.subplot(4,1,1)
plt.plot(dfBehavior.Time, dfBehavior.F0, color='b',label='FlagAvoid')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')
#plt.legend(bbox_to_anchor=(1,1), borderaxespad=0)

plt.subplot(4,1,2)
plt.plot(dfBehavior.Time, dfBehavior.F1, color='g', label='FlagGoLoad')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(4,1,3)
plt.plot(dfBehavior.Time, dfBehavior.F2, color='r', label='FlagForage')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.subplot(4,1,4)
plt.plot(dfBehavior.Time, dfBehavior.F4, color='m', label='FlagGo4Walle')
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.legend(loc='best')

plt.savefig('images/behaviorSubplotsn.png')
plt.show()


###############
###############
# POSITION VS BEHAVIOR

dfPosition = pd.read_table('robotPosition', delim_whitespace=True, names=('Time','X','Y','Orientation'))
i=0
xF0,yF0,xF1,yF1,xF2,yF2,xF3,yF3,xF4,yF4 = [],[],[],[],[],[],[],[],[],[]
binsB0,binsB1,binsB2,binsB3,binsB4 = 0,0,0,0,0

while(i < dfBehavior.Time.count()):
    if(dfBehavior.F0[i] == 1):
        binsB0+=1
        xF0.append(dfPosition.X[i])
        yF0.append(dfPosition.Y[i])
    elif(dfBehavior.F1[i] == 1):
        binsB1+=1
        xF1.append(dfPosition.X[i])
        yF1.append(dfPosition.Y[i])
    elif(dfBehavior.F2[i] == 1):
        binsB2+=1
        xF2.append(dfPosition.X[i])
        yF2.append(dfPosition.Y[i])
    elif(dfBehavior.F4[i] == 1):
        binsB4+=1
        xF4.append(dfPosition.X[i])
        yF4.append(dfPosition.Y[i])
    i+=1

#print(binsB4)
s=1

plt.scatter(xF0,yF0,color='b',label='Avoid',s=s)
plt.scatter(xF1,yF1,color='g',label='GoLoad',s=s)
plt.scatter(xF2,yF2,color='r',label='Forage',s=s)
plt.scatter(xF4,yF4,color='m',label='Go4Walle',s=s)
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('Position vs. Behavior')
plt.legend(loc='best')
plt.savefig('images/posBehaviors.png')
plt.show()


##############
##############
# BARS


names = ['Avoid', 'GoLoad', 'Forage', 'Go4Walle']
values = np.array([binsB0,binsB1,binsB2,binsB4])
plt.bar([1,2,3,4], values, tick_label = names, color=['b','g','r','m'])
plt.grid(color = '0.7', linestyle= '--', linewidth=.5)
plt.xlabel('Behavior')
plt.ylabel('Timesteps')
plt.title('Timesteps of each Behavior ')
plt.savefig('images/barsBehaviors.png')
plt.show()