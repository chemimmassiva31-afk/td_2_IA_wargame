from controller import Robot

MAX_SPEED = 6.28
RAW_MIN = 80.0
RAW_MAX = 600.0


def norm(raw):
    p = (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)
    if p < 0:
        p = 0.0
    if p > 1:
        p = 1.0
    return p


def trap(x, a, b, c, d):
    if b <= x <= c:
        return 1.0
    if a < x < b:
        return (x - a) / (b - a)
    if c < x < d:
        return (d - x) / (d - c)
    return 0.0


IN = {
    "E": {
        "TROP_LOIN":   (0.00, 0.00, 0.15, 0.35),
        "BONNE":       (0.15, 0.45, 0.45, 0.75),
        "TROP_PROCHE": (0.55, 0.80, 1.00, 1.00),
    },
    "AL": {
        "DIVERGE":   (-1.00, -1.00, -0.15, 0.00),
        "PARALLELE": (-0.10, 0.00, 0.15, 0.28),
        "CONVERGE":  (0.15, 0.32, 1.00, 1.00),
    },
    "F": {
        "LIBRE":  (0.00, 0.00, 0.05, 0.25),
        "PROCHE": (0.10, 0.40, 1.00, 1.00),
    },
}


def fuzzifier(p_lat, p_avd, p_ard, p_avg, p_front):
    E = {}
    for mot, (a, b, c, d) in IN["E"].items():
        E[mot] = trap(p_lat, a, b, c, d)

    alignement = p_avd - p_ard
    AL = {}
    for mot, (a, b, c, d) in IN["AL"].items():
        AL[mot] = trap(alignement, a, b, c, d)

    devant = max(p_front, 0.5 * p_avd, p_avg)
    F = {}
    for mot, (a, b, c, d) in IN["F"].items():
        F[mot] = trap(devant, a, b, c, d)

    return E, AL, F


def arrondi(dico):
    return {mot: round(valeur, 2) for mot, valeur in dico.items()}


robot = Robot()
timestep = int(robot.getBasicTimeStep())

ps = {}
for i in range(8):
    nom = "ps" + str(i)
    ps[nom] = robot.getDevice(nom)
    ps[nom].enable(timestep)

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

t = 0
while robot.step(timestep) != -1:

    raw = {}
    for nom in ps:
        raw[nom] = ps[nom].getValue()

    p = {}
    for nom in raw:
        p[nom] = norm(raw[nom])

    E, AL, F = fuzzifier(p["ps2"], p["ps1"], p["ps3"], p["ps6"], max(p["ps0"], p["ps7"]))

    if t % 15 == 0:
        print("E =", arrondi(E), "| AL =", arrondi(AL), "| F =", arrondi(F))

    left.setVelocity(0.0)
    right.setVelocity(0.0)
    t += 1