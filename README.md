# - IGUS Student -

### 1. Workspace vorbereiten

```bash
# In das Workspace-Verzeichnis navigieren
cd ~/rebel_student

# Den Workspace bauen
colcon build --symlink-install

# Setup-Skript laden
source install/setup.bash
```

### 2. Robot und MoveIt starten

Bevor du dein Programm startest, musst du den Roboter und MoveIt starten:

#### Simulation

```bash
# Terminal 1: Robotersimulation starten
ros2 launch igus_rebel_moveit_config igus_rebel_simulated.launch.py

# Terminal 2: Dein Student-Programm
ros2 run igus_student student_control
```

#### Echter Roboter

```bash
# Terminal 1: Roboter verbinden
ros2 launch igus_rebel rebel.launch.py

# Terminal 2: Motion planner starten
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py

# Terminal 3: Dein Student-Programm
ros2 run igus_student student_control
```

## Erste Schritte

### Wo schreibe ich Code?

```bash
└── src
    ├── igus_student
    │   ├── igus_student
    │   │   └── student_control.py
```

ab Zeile 80

## Grundlegende Funktionen

##### `move_and_wait(x, y, z, roll, pitch, yaw)`

Bewegt den Roboter zu einer bestimmten Position und wartet, bis er dort angekommen ist.

**Parameter:**
- `x, y, z` (float): Kartesische Koordinaten in Metern (relativ zur Basis)
- `roll, pitch, yaw` (float): Orientierung in Radiant (Euler-Winkel)

**Rückgabewert:**
- `True` wenn erfolgreich, `False` bei Fehler

**Beispiel:**
```python
# Roboter zu Position (0.4, 0.2, 0.3) mit Orientierung (π, 0, 0) bewegen
success = robot.move_and_wait(0.4, 0.2, 0.3, 3.14, 0.0, 0.0)
if success:
    print("Bewegung erfolgreich!")
else:
    print("Bewegung fehlgeschlagen!")
```

## Fehlerbehebung

### Problem: "MoveGroup server not available"

**Lösung:** Stelle sicher, dass der Motion planner läuft:
```bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py
```

### Problem: Roboter bewegt sich nicht

**Lösungen:**
Überprüfe, ob die Zielposition im Arbeitsbereich des Roboters liegt

### Problem: Timeout beim Warten auf Bewegung

Dies tritt auf, wenn der Roboter die Zielposition nicht erreichen kann. Mögliche Ursachen:
- Position ist außerhalb des Arbeitsbereichs
- Kollision mit Hindernis erkannt
- Roboter kann die Orientierung nicht erreichen

## Koordinatensystem

### Kartesische Koordinaten (X, Y, Z)

- **X-Achse:** Vorwärts/Rückwärts (in Meter)
- **Y-Achse:** Links/Rechts (in Meter)
- **Z-Achse:** Oben/Unten (in Meter)

### Orientierung (Roll, Pitch, Yaw)

Die Orientierung wird mit Euler-Winkeln in Radiant definiert:
- **Roll:** Rotation um die X-Achse
- **Pitch:** Rotation um die Y-Achse
- **Yaw:** Rotation um die Z-Achse

**Wichtige Konstanten:**
```python
import math

pi = math.pi
# Roll = π (180°) bedeutet: Roboter zeigt nach unten
# Roll = 0, Pitch = 0, Yaw = 0: Roboter zeigt nach oben
```

## Weitere Ressourcen

- **Konstanten:** Siehe `igus_student/constants.py`
- **Quellcode:** `igus_student/student_control.py`

Viel Erfolg beim Programmieren!
