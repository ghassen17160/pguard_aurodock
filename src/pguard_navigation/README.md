# pguard_navigation

Planificateur global PRM (OMPL) + planificateur local DWA, portés depuis une
version testée sur TurtleBot3 Waffle vers le robot **pguard**.

## Ce qui a été corrigé par rapport à la version waffle

Voir le bloc `CHANGELOG` en haut de `src/prm_nav_node.cpp` pour le détail. En
résumé :

1. **Bug de yaw** : le cap utilisé pour projeter les obstacles au premier
   point de chaque trajectoire DWA était figé à 0.0 au lieu du cap réel du
   robot.
2. **Perf DWA** : les ~720 rayons du scan pguard (`pointcloud_to_laserscan`,
   `angle_increment=0.00872`) étaient reprojetés pour **chacune** des ~250
   trajectoires candidates testées à chaque cycle de contrôle (50 ms). C'est
   maintenant fait une seule fois par cycle.
3. **Sous-échantillonnage du scan** (`scan_stride`) pour garder de la marge.
4. Paramètres `dwa_v_step` / `dwa_w_step` exposés (avant : constantes
   hardcodées) pour pouvoir retoucher la résolution de recherche du DWA sans
   recompiler.
5. Nettoyage de variables mortes, code dupliqué (double appel
   `publish_costmap()` etc.) supprimé.
6. **Compatibilité `spawn_pguard.launch.py` / `mapping.launch.py`** :
   l'ancienne version du DWA calculait la position des obstacles avec une
   formule géométrique approximative (`current_x + range*cos(angle+yaw)`),
   ce qui suppose que le scan est exactement centré sur `base_link` et que
   la pose odométrique correspond exactement à `base_link`. Rien ne
   garantit ça côté pguard (le xacro a `base_footprint` ET `base_link`
   comme segments distincts). Le DWA utilise maintenant exactement le même
   mécanisme TF que le costmap (`tf_buffer_->lookupTransform` vers
   `map_frame_`), donc ça marche quelle que soit la chaîne TF réelle,
   indépendamment de `target_frame` dans `pointcloud_to_laserscan`. Bonus :
   les points ne sont recalculés qu'à l'arrivée d'un nouveau scan (~10 Hz),
   plus à chaque cycle de contrôle (20 Hz) — donc plus rapide qu'avant.
7. **QoS du `/scan`** : `pointcloud_to_laserscan_node` publie en général
   avec une QoS "sensor data" (`best_effort`). Le node s'abonne maintenant
   explicitement en `best_effort`/`volatile` pour éviter de ne recevoir
   aucun message par incompatibilité de QoS.
8. `goal_callback` comparait le frame_id du goal reçu à `"map"` en dur au
   lieu du paramètre configurable `map_frame_`. Corrigé.
9. Timeout TF de `odom_callback` élargi à 2.0s (au démarrage, `map->odom`
   via `slam_toolbox` peut mettre quelques instants à apparaître après
   l'enchaînement `spawn_pguard.launch.py` puis `mapping.launch.py`).


## Limitation connue (non corrigée, à surveiller)

La fenêtre de costmap est calculée **une seule fois**, centrée sur la
position de départ du robot, et sa taille reste fixe pendant toute
l'exécution (`map_width * resolution` x `map_height * resolution` mètres).
Si votre environnement pguard est plus grand que cette fenêtre, tout goal ou
obstacle en dehors sera silencieusement ignoré ou refusé (log
`out of bounds`). Dimensionnez `map_width`/`map_height`/`resolution` dans
`config/pguard_nav_params.yaml` en fonction de la taille réelle de votre
zone de test. Une carte qui suit dynamiquement le robot serait l'étape
suivante si votre zone est très grande.

## Installation

### 1. Dépendances système

```bash
sudo apt update
sudo apt install libompl-dev ros-$ROS_DISTRO-tf2-geometry-msgs ros-$ROS_DISTRO-visualization-msgs
```

### 2. Copier le package dans votre workspace

```bash
cp -r pguard_navigation ~/Desktop/2206/git/pguard1_ws/src/
```

### 3. Build

```bash
cd ~/Desktop/2206/git/pguard1_ws
colcon build --symlink-install --packages-select pguard_navigation
source install/setup.bash
```

Si CMake ne trouve pas OMPL (`find_package(ompl REQUIRED)` échoue), vérifiez
avec :
```bash
dpkg -L libompl-dev | grep cmake
```
et essayez la variante majuscule `find_package(OMPL REQUIRED)` /
`${OMPL_INCLUDE_DIRS}` / `${OMPL_LIBRARIES}` dans `CMakeLists.txt` si besoin
(dépend de la distro ROS).

## Calibration AVANT de lancer sur pguard

Éditez `config/pguard_nav_params.yaml` et remplacez ces valeurs par les
vraies caractéristiques de pguard :

| Paramètre | Comment le déterminer |
|---|---|
| `max_linear_vel`, `max_angular_vel` | fichier de config de votre contrôleur `diff_drive_controller` (ou équivalent) dans `pearlguard_description` |
| `safety_buffer` | demi-largeur du châssis pguard (regardez les dimensions dans le xacro) + marge de sécurité (~10-15 cm) |
| `inflation_radius` | `safety_buffer` + marge supplémentaire pour laisser de la place à la replanification |
| `map_width` / `map_height` / `resolution` | taille réelle de votre zone de test en mètres, divisée par la résolution voulue |

Commande utile pour retrouver les dimensions du châssis :
```bash
grep -ri "box\|cylinder\|radius\|length" ~/Desktop/2206/git/pguard1_ws/src/pearlguard_description/urdf/robot_pguard_x.xacro
```

## Lancement (3 terminaux)

```bash
# Terminal 1
ros2 launch pearlguard_description spawn_pguard.launch.py

# Terminal 2 (attendre que gazebo + robot soient bien spawnés)
ros2 launch pearlguard_description mapping.launch.py

# Terminal 3
ros2 launch pguard_navigation navigation.launch.py
```

Dans RViz, utilisez l'outil **"2D Nav Goal"** (publie sur
`/move_base_simple/goal`).

## Debug

```bash
ros2 topic list                     # /odom /scan /cmd_vel doivent exister sans namespace
ros2 topic echo /odom --once        # vérifier frame_id="odom" (ou équivalent)
ros2 run tf2_tools view_frames      # vérifier la chaîne map -> odom -> base_link
ros2 topic echo /prm_path           # vérifier que le PRM trouve un chemin
ros2 topic hz /cmd_vel              # vérifier que le control_timer_ (50ms -> ~20Hz) tient le rythme
```

Si `ros2 topic hz /cmd_vel` tombe sous ~15-18 Hz, augmentez `scan_stride`,
`dwa_v_step` ou `dwa_w_step` dans le fichier de config pour réduire la
charge de calcul du DWA.
