
# coding: utf-8

# In[85]:

import math

#69 cm from the edge of from the table to the edge of the frame, 133 cm across the screen

def calculateTargetXAndY(Cans, Target):
    NotHittable = Cans
    NotHittable.remove(Target)
    gravity = 9.8;
    test=False
    velocities = []
    max_vel = 53 #10*5.3
    for veliter in range(max_vel,0,-1):
        velocities.append(veliter/10)
    for theta in range(90):
        #for vel in velocties:
        #This can do a continuous range decreasing from our max velocity
         for vel in velocities:
            max_time = (2 * vel * math.sin(theta)) / gravity
            max_time = math.ceil(100*max_time)
            time_step = []
            for timeIter in range(max_time):
                time_step.append(timeIter/100)
            for t in time_step:
                test = True
                x = vel * math.cos(math.radians(theta)) * t
                y = vel * math.sin(math.radians(theta)) * t - (1/2) * (gravity * (t ** 2))
                #Convert x y into cm
                #Turn it into whole numbers and subtract distance between camera and frame
                x=round(x*100)-69
                y=round(y*100)
                if (doesXYIntercept(x,y,NotHittable)):
                    break
                elif (doesXYCollide(x,y,Target)):
                    #print(vel, theta)
                    finalx,finaly=calculateBotPosition(vel, theta)
                    return finalx*100,finaly*100
    print('This position cant be hit')
    return False
    

def calculateBotPosition(vel, theta):
    #These two need to be find, mass of ball is 0.07 grams and the change of the spring with a 983 grams was 4 cm
    springk = 0.983/0.04
    mass = 0.007
    changeX = (((vel ** 2) * mass) / springk)**0.5
    x = changeX * math.cos(math.radians(theta))
    y = changeX * math.sin(math.radians(theta))
    return x, y

def doesXYIntercept(x,y,Obsticles):
    for i in Obsticles:
        if (x,y) == i:
            return True
    return False

def doesXYCollide(x,y,Target):
    if x < Target[0]+5 and y < Target[1]+5:
        if x > (Target[0]-5) and y > Target[1]-5:
            print(x,y,Target[0],Target[1])
            return True
    return False


# In[86]:

#Cans=[(850,14)]
#Target=(850,14)

#x,y=calculateTargetXAndY(Cans, Target)


# In[84]:

#Get max velocity, we can change this
#CX=((-0.13+0.21)**2+(0.01+0.03)**2)**0.5
#springk = 0.983/0.04
#mass = 0.007
#print(CX)


# In[27]:

#((springk*(CX)**2)/mass)**0.5


# In[ ]:



