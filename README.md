## Instalacja
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/tadezegiusz/ur5_click_project.git
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

## Uruchomienie

### 1. Przygotowanie robota (PolyScope)
1. Uruchom robota UR5.
   ```bash
   ros2 run ur_client_library start_ursim.sh -m ur5
   ```
2. W PolyScope wgraj program z **External Control**.
3. Ustaw adres IP komputera oraz port.
4. Uruchom program.

---

### 2. Zbudowanie i uruchomienie
W terminalu:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select ur5_click_project
source install/setup.bash
ros2 launch ur5_click_project ur5_rviz.launch.py
```

Zostanie uruchomiona wizualizacja RViz oraz panel sterowania.

---

## Obsługa
- kliknięcie w górnej części panelu → ruch ramienia w górę
- kliknięcie w dolnej części panelu → ruch ramienia w dół

---
