# - IGUS Student -

### 1. Workspace vorbereiten

```bash
# In das Workspace-Verzeichnis navigieren
cd ~/rebel_student

# Den Workspace bauen
colcon build --symlink-install

# --symlink-install erzeugt einen symbolischen Link zu den src-Dateien
# ohne --symlink-install muss man nach jeder Änderung erneut über colcon build bauen

# Workspace laden
igus
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
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true

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

~ ab Zeile 80

## Grundlegende Funktionen

### `move_and_wait(x, y, z, roll, pitch, yaw)`
```python 
move_and_wait(x, y, z, roll, pitch, yaw)
```

Bewegt den Roboter zu einer bestimmten Position und wartet, bis er dort angekommen ist.

**Parameter:**
- `x, y, z` (float): Kartesische Koordinaten in Metern (relativ zur Basis)
- `roll, pitch, yaw` (float): Orientierung in Radiant (Euler-Winkel)

**Beispiel:**
```python
# Roboter zu Position (0.4, 0.2, 0.3) mit Orientierung (~π, 0, 0) bewegen
_robot.move_and_wait(0.4, 0.2, 0.3, 3.14, 0.0, 0.0)
```

### `igus.move_ee_vertical(node: Node, delta_z: float = 0.1)`
```python
igus.move_ee_vertical(node: Node, delta_z: float = 0.1)
```

Bewegt den Roboter 0.1m in z-Richtung.

**Parameter:**
- `node` (Node): Node des zu steuernden Objektes.
- `delta_z` (float): Distanz in Meter, welche in Z-Richtung verfahren werden soll.

**Beispiel:**
```python
# Roboter um 0.1m nach oben fahren
igus.move_ee_vertical(_robot, delta_z=0.1)
```

## Fehlerbehebung

### Problem: NOT-AUS

**Lösung:** NOT-AUS herausdrehen. `rebel.launch.py` neustarten.

### Problem: "move_group server not available"

**Lösung:** Stelle sicher, dass der Motion planner läuft:
```bash
ros2 launch igus_rebel_moveit_config igus_rebel_motion_planner.launch.py use_gui:=true
```

### Problem: Roboter bewegt sich nicht

**Lösung:**
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

Viel Erfolg beim Programmieren!
