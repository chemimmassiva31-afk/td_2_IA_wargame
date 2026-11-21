from controller import Robot

WALL_SIDE = "right"
MAX_SPEED = 6.28
CRUISE = 0.6
TURN = 0.5
RAW_MIN = 80.0
RAW_MAX = 600.0
AFFICHAGE = True


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


OUT_ANGLE = {
    "DROITE_FORT": (-45, -45, -45, -20),
    "DROITE":      (-35, -18, -18, 0),
    "TOUT_DROIT":  (-10, 0, 0, 10),
    "GAUCHE":      (0, 18, 18, 35),
    "GAUCHE_FORT": (20, 45, 45, 45),
}

OUT_SPEED = {
    "ARRET":  (0.0, 0.0, 0.0, 0.25),
    "LENT":   (0.0, 0.2, 0.2, 0.5),
    "MOYEN":  (0.2, 0.5, 0.5, 0.8),
    "RAPIDE": (0.5, 1.0, 1.0, 1.0),
}

TABLE_ANGLE = {
    ("TROP_PROCHE", "DIVERGE"):   "TOUT_DROIT",
    ("TROP_PROCHE", "PARALLELE"): "GAUCHE",
    ("TROP_PROCHE", "CONVERGE"):  "GAUCHE_FORT",
    ("BONNE",       "DIVERGE"):   "DROITE",
    ("BONNE",       "PARALLELE"): "TOUT_DROIT",
    ("BONNE",       "CONVERGE"):  "GAUCHE",
    ("TROP_LOIN",   "DIVERGE"):   "DROITE_FORT",
    ("TROP_LOIN",   "PARALLELE"): "DROITE",
    ("TROP_LOIN",   "CONVERGE"):  "TOUT_DROIT",
}


def regles_angle(E, AL, F):
    regles = []
    for (e, a), sortie in TABLE_ANGLE.items():
        degre = min(E[e], AL[a], F["LIBRE"])
        regles.append((sortie, degre))
    regles.append(("GAUCHE_FORT", F["PROCHE"]))
    return regles


def regles_vitesse(E, AL, F):
    return [
        ("ARRET",  F["PROCHE"]),
        ("LENT",   min(F["LIBRE"], E["TROP_PROCHE"])),
        ("MOYEN",  min(F["LIBRE"], E["TROP_LOIN"])),
        ("RAPIDE", min(F["LIBRE"], E["BONNE"], AL["PARALLELE"])),
        ("MOYEN",  min(F["LIBRE"], E["BONNE"], max(AL["CONVERGE"], AL["DIVERGE"]))),
    ]


def centroide(regles, termes, lo, hi, defaut=0.0, n=201):
    num = 0.0
    den = 0.0
    for i in range(n):
        u = lo + (hi - lo) * i / (n - 1)
        mu = 0.0
        for sortie, degre in regles:
            a, b, c, d = termes[sortie]
            mu = max(mu, min(degre, trap(u, a, b, c, d)))
        num += u * mu
        den += mu
    if den < 1e-9:
        return defaut
    return num / den


def vers_roues(angle, vitesse):
    V = vitesse * CRUISE * MAX_SPEED
    K = TURN * MAX_SPEED * (angle / 45.0)
    vg = V - K
    vd = V + K
    s = max(1.0, abs(vg) / MAX_SPEED, abs(vd) / MAX_SPEED)
    return vg / s, vd / s


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

if WALL_SIDE == "right":
    LAT, AVD, ARD, AVG = "ps2", "ps1", "ps3", "ps6"
    SIGNE = 1
else:
    LAT, AVD, ARD, AVG = "ps5", "ps6", "ps4", "ps1"
    SIGNE = -1

mesures = []
for _ in range(10):
    if robot.step(timestep) == -1:
        break
    mesures.append(ps[LAT].getValue())
if len(mesures) > 0:
    v = sum(mesures) / len(mesures)
    if v > RAW_MIN + 100:
        RAW_MAX = RAW_MIN + (v - RAW_MIN) / 0.45
    print("Calibration : capteur lateral =", round(v), "-> RAW_MAX =", round(RAW_MAX))

t = 0
while robot.step(timestep) != -1:

    raw = {}
    for nom in ps:
        raw[nom] = ps[nom].getValue()

    p = {}
    for nom in raw:
        p[nom] = norm(raw[nom])

    E, AL, F = fuzzifier(p[LAT], p[AVD], p[ARD], p[AVG], max(p["ps0"], p["ps7"]))

    angle = centroide(regles_angle(E, AL, F), OUT_ANGLE, -45, 45, 0.0)
    vitesse = centroide(regles_vitesse(E, AL, F), OUT_SPEED, 0, 1, 0.3)

    vg, vd = vers_roues(SIGNE * angle, vitesse)
    left.setVelocity(vg)
    right.setVelocity(vd)

    if AFFICHAGE and t % 15 == 0:
        print({nom: round(v) for nom, v in raw.items()},
              "| angle =", round(angle, 1), "| vitesse =", round(vitesse, 2),
              "| roues =", round(vg, 2), round(vd, 2))
    t += 1