from controller import Robot

robot=Robot()
timestep=int(robot.getBasicTimeStep())

ps={}
for n in ['ps1', 'ps2', 'ps5', 'ps6']:
   ps[n] = robot.getDevice(n)
   ps[n].enable(timestep)
   
   
left= robot.getDevice('left wheel motor')
right= robot.getDevice('right wheel motor')
left.setPosition(float('inf'))
right.setPosition(float('inf'))
left.setVelocity(0)
right.setVelocity(0)

while robot.step(timestep) != -1:
      dist_droite = max(ps['ps1'].getValue() , ps['ps2'].getValue())
      dist_gauche = max(ps['ps5'].getValue() , ps['ps6'].getValue())
      print(dist_gauche,dist_droite)
      
      
      left.setVelocity(1.0)
      right.setVelocity(3.0)