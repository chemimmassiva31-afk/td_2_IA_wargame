def norm(raw):
    return min(1.0, max(0.0, (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)))


robot = Robot()
timestep = int(robot.getBasicTimeStep())

ps = {f"ps{i}": robot.getDevice(f"ps{i}") for i in range(8)}
for s in ps.values():
    s.enable(timestep)

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
for m in (left, right):
    m.setPosition(float("inf"))
    m.setVelocity(0.0)

t = 0
while robot.step(timestep) != -1:
    raw = {nom: s.getValue() for nom, s in ps.items()}
    p = {nom: norm(v) for nom, v in raw.items()}           # proximités normalisées
    if t % 15 == 0:
        print({nom: round(v) for nom, v in raw.items()}, "->",
              {nom: round(v, 2) for nom, v in p.items()})

    left.setVelocity(0.0)                        
    right.setVelocity(0.0)
    t += 1