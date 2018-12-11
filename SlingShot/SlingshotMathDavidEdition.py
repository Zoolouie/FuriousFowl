
# coding: utf-8

# In[17]:

import math

#69 cm from the edge of from the table to the edge of the frame, 133 cm across the screen

def calculateTargetXAndY(Cans, Target):
    NotHittable = Cans
    NotHittable.remove(Target)
    #velocities = [100, 50, 20] #CHANGE THESE VALUES
    gravity = 9.8;
    test=False
    xl=[]
    yl=[]
    for theta in range(90):
        #for vel in velocties:
        #This can do a continuous range decreasing from our max velocity
         for vel in range(10,0,-1):
            max_time = math.ceil((2 * vel * math.sin(theta)) / gravity)
            time_step = []
            for timeIter in range(10*max_time):
                time_step.append(timeIter/10)
            for t in time_step:
                test = True
                x = vel * math.cos(theta) * t
                y = vel * math.sin(theta) * t - (1/2) * (gravity * (t ** 2))
                #Convert x y into cm
                #Turn it into whole numbers and subtract distance between camera and frame
                x=round(x*100)-69
                y=round(y*100)
                print(x,y)
                xl.append(x)
                yl.append(y)
                if (doesXYIntercept(x,y,NotHittable)):
                    break
                elif (doesXYCollide(x,y,Target)):
                    print(vel, theta)
                    finalx,finaly=calculateBotPosition(vel, theta)
                    return finalx*100,finaly*100,theta
            #if test==True:
                #return xl,yl
    print('This position cant be hit')
    return False
    

def calculateBotPosition(vel, theta):
    #These two need to be find, mass of ball is 0.07 grams and the change of the spring with a 983 grams was 4 cm
    springk = 0.983/0.04
    mass = 0.007
    changeX = (((vel ** 2) * mass) / springk)**0.5
    x = changeX * math.cos(theta)
    y = changeX * math.sin(theta)
    return x, y

def doesXYIntercept(x,y,Obsticles):
    for i in Obsticles:
        if (x,y) == i:
            return True
    return False

def doesXYCollide(x,y,Target):
    if (x,y)==Target:
        return True
    else:
        return False


# In[2]:

Test,Test2=calculateBotPosition(30,20)


# In[18]:

print(Test,Test2)


# In[19]:

Cans=[(850,14)]
Target=(850,14)
x,y,theta=calculateTargetXAndY(Cans, Target)


# In[20]:

print(x,y)


# In[56]:

TimeShtuff=[]
for timeIter in range(10*2):
    TimeShtuff.append(timeIter/10)


# In[54]:

print(TimeShtuff)


# In[70]:

round(5.4)


# In[5]:

import matplotlib.pyplot as plt
plt.plot(x, y, 'ro')
plt.show()


# In[ ]:



