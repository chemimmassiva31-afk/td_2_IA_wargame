
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialisation des capteurs
ps = {}

for nom in ["ps1", "ps2", "ps5", "ps6"]:
    ps[nom] = robot.getDevice(nom)
    ps[nom].enable(timestep)

# Initialisation des moteurs
left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")

left.setPosition(float("inf"))
right.setPosition(float("inf"))

left.setVelocity(0)
right.setVelocity(0)

while robot.step(timestep) != -1:

    # Lecture des capteurs
    distance_droite = max(
        ps["ps1"].getValue(),
        ps["ps2"].getValue()
    )

    distance_gauche = max(
        ps["ps5"].getValue(),
        ps["ps6"].getValue()
    )

    print("Droite :", distance_droite,
          "Gauche :", distance_gauche)

    
    left.setVelocity(2.0)
    right.setVelocity(2.0)
