#Global Variables Mass, Gravity and Spring Constant

def calculateTargetXAndY:
	velocities = [100, 50, 20];
	gravity = 9.8;
	for theta in range(90):
		for vel in velocties:
			max_time = (2 * vel * sin(theta)) / gravity
			for t in max_time:
				x = vel * cos(theta) * t
				y = vel * sin(theta) * t - (1/2) * (gravity * (t ** 2))
				if (doesXYIntercept()):
					break
				else if (doesXYCollide()):					
					return calculateBotPosition(vel, theta)

def calculateBotPosition(vel, theta):
	changeX = sqrt((v ** 2) * mass / springk)
	x = changeX * cos(theta)
	y = changeX * sin(theta)
	return x, y








#iterate through angles by change in 1 degree (0-90)
#iterate through velocities backwards starting at max velocity 