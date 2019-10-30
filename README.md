Auto-Organisation avec couverture de point d'intérêt
======================================================

Package AAU_RNG_MULTI_ROBOT
---------------------------
<div align="justify">
Le package  `aau_rng_multi_robot` est un package ROS qui est constitué de deux n&oelig;uds (i.e., nodes ROS). Brièvement, le premier node apporte la communication réseau et l'auto-organisation des robots à travers l'algorithme de construction d'un RNG Local (i.e., <strong>Local Relative Neighborhood Graph - LRNG</strong>). Il est identifiable par le nom `aau_rng_communication` dans la liste des nodes ROS (voir avec la commande `rosnode list`). Le second node décrit l'implémentation de l'algorithme de couverture de Point d'Intérêt (i.e. <strong>PoI</strong>), aussi appelé `poi_coverage`. Le package est installable dans un `catkin workspace : catkin_ws/src`.
</div>

### Préalable###
<div align="justify">
Ce package est destiné à être exploité sur des robots <strong>Turtlebot2</strong>. Pour naviguer dans l'environement, étant donné qu'on utilise le module <code>turtlebot_navaigation</code>, le second node ou <strong>PoI</strong> a besoin une map qui doit-être installée dans un répertoire ```catkin_ws/src/maps/```. Donc, si ce répertoire n'existe pas, vous devriez le créer et copier les fichiers de votre map (e.g., ```map_name.yaml``` et ```map_name.pgm```) à l'intérieur de ce répertoire. Vous devez également déclarer une variable d'environement  dans le fichier `bashrc` de votre environement de travail (i.e., `/home`) comme suit :
</div>

```bash
export TURTLEBOT_MAP_FILE=~/catkin_ws/src/maps/map_name.yaml

```
<div align="justify">
Ceci est une variable globale du module de navigation des robots <strong>Turtlebot2</strong> qui définit le chemin de la map à utiliser lors une navigation avec map (i.e., navigation assistée). Vous devez également modifier le fichier `amcl_demo.launch` pour y rajouter la position initiale  `x` et `y` qui répresente la position de son docker de charge. Vous devez naviguer jusqu'au module de navigation et modifier le à travers cette serie de commande shell.
</div>

```bash
$ roscd turtlebot_navigation/launch/
$ sudo vim/gedit/nano amcl_demo.launch  # Utilisez votre éditeur favoris: vim ou gedit ou nano
```

Vous aurez une sortie pareil :
```bash
<launch>
  <!-- 3D sensor -->
  <arg name="3d_sensor" default="$(env TURTLEBOT_3D_SENSOR)"/>  <!-- r200, kinect, asus_xtion_pro -->
  <include file="$(find turtlebot_bringup)/launch/3dsensor.launch">
    <arg name="rgb_processing" value="false" />
    <arg name="depth_registration" value="false" />
    <arg name="depth_processing" value="false" />

    <!-- We must specify an absolute topic name because if not it will be prefixed by "$(arg camera)".                    Probably is a bug in the nodelet manager: https://github.com/ros/nodelet_core/issues/7 -->
    <arg name="scan_topic" value="/scan" />
  </include>

  <!-- Map server -->
  <arg name="map_file" default="$(env TURTLEBOT_MAP_FILE)"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- AMCL -->
  <arg name="custom_amcl_launch_file" default="$(find turtlebot_navigation)/launch/includes/amcl/$(arg 3d_sensor)_amcl.launch.xml"/>
  <arg name="initial_pose_x" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
  <arg name="initial_pose_y" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
  <arg name="initial_pose_a" default="0.0"/>
  <include file="$(arg custom_amcl_launch_file)">
    <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
    <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
    <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>
  </include>

  <!-- Move base -->
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/$(arg 3d_sensor)_costmap_params.yaml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml">
    <arg name="custom_param_file" value="$(arg custom_param_file)"/>
  </include>

</launch>
```
Vous devez modifier les deux lignes suivantes :
```bash
<arg name="initial_pose_x" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
<arg name="initial_pose_y" default="0.0"/> <!-- Use 17.0 for willow's map in simulation -->
```
<div align="justify">
en y ajoutant la position initiale du robot lorsqu'il est en charge au niveau du docker de charge. ==++Note bien que, cette position initiale peut-être aussi sa position initiale de départ sans docker de charge++==. Par exemple dans notre cas, nous avons un robot qui est initialement positionné à `(-0.94, -4.20)` dans notre map, donc son fichier `amcl_demo.launch` aura la sortie suivante :
</div>

```bash
<launch>
  <!-- 3D sensor -->
  <arg name="3d_sensor" default="$(env TURTLEBOT_3D_SENSOR)"/>  <!-- r200, kinect, asus_xtion_pro -->
  <include file="$(find turtlebot_bringup)/launch/3dsensor.launch">
    <arg name="rgb_processing" value="false" />
    <arg name="depth_registration" value="false" />
    <arg name="depth_processing" value="false" />

    <!-- We must specify an absolute topic name because if not it will be prefixed by "$(arg camera)".
     Probably is a bug in the nodelet manager: https://github.com/ros/nodelet_core/issues/7 -->
    <arg name="scan_topic" value="/scan" />
  </include>

  <!-- Map server -->
  <arg name="map_file" default="$(env TURTLEBOT_MAP_FILE)"/>
  <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />

  <!-- AMCL -->
  <arg name="custom_amcl_launch_file" default="$(find turtlebot_navigation)/launch/includes/amcl/$(arg 3d_sensor)_amcl.launch.xml"/>
 <arg name="initial_pose_x" default="-0.94"/> <!-- (0.0) Use 17.0 for willow's map in simulation -->
 <arg name="initial_pose_y" default="-4.20"/> <!-- (0.0) Use 17.0 for willow's map in simulation -->
  <arg name="initial_pose_a" default="0.0"/>
  <include file="$(arg custom_amcl_launch_file)">
    <arg name="initial_pose_x" value="$(arg initial_pose_x)"/>
    <arg name="initial_pose_y" value="$(arg initial_pose_y)"/>
    <arg name="initial_pose_a" value="$(arg initial_pose_a)"/>
  </include>

  <!-- Move base -->
  <arg name="custom_param_file" default="$(find turtlebot_navigation)/param/$(arg 3d_sensor)_costmap_params.yaml"/>
  <include file="$(find turtlebot_navigation)/launch/includes/move_base.launch.xml">
    <arg name="custom_param_file" value="$(arg custom_param_file)"/>
  </include>

</launch>

```

Pour l'installation de ROS  sur le PC (i.e., Contrôleur) du robot, vous pouvez vous référer à [Learn TurtleBot and ROS](http://learn.turtlebot.com/).


### Structure hiérarchique du répertoire ###
Le contenu du package est organisé comme suivant :

```
aau_rng_multi_robot
├── CMakeLists.txt
├── include
│   └── aau_rng_multi_robot
├── launch
│   ├── aau_rng_adhoc_communication.launch
│   └── turtlebot_setup.launch
├── msg
│   ├── PointM.msg
│   ├── RecvString.msg
│   └── Robot.msg
├── package.xml
├── script
│   ├── aau_rng_communication.py
│   ├── broadcast_api.py
│   ├── data_structures.py
│   ├── get_netifaces_addresses.py
│   ├── map_navigation.py
│   ├── poi_coverage.py
│   ├── robot_data.py
│   └── utils.py
├── src
└── srv
    ├── MaxDistance.srv
    └── ShutDown.srv

```

<div align="justify">
Les fichiers `CMakeLists.txt` et `package.xml` sont inhérents à tout package ROS. Vous pouvez y jeter un &oelig;il  sur le contenu de ces deux fichiers. Les répertoires `msg`, `srv`, `launch`, `src` et `script` sont respectivement les répertoires des messages, des services, des fichiers de lancement, des n&oelig;uds (i.e. node) ou modules écrits en `C++` et les n&oelig;uds écrits en `Python`. Le répertoire `include` sert de répertoire de stockage des fichiers entêtes des modules écrits en `C/C++`.
</div>


##### Répertoire MSG
---------------------

<div align="justify">
Ce répertoire contient les messages crées (ou qui seront crées) et exploités (ou qui seront exploités) dans les modules du package. Actuellement, il y a trois messages déclarés, mais aucun n'est utilisé, mais pourront faire l'objet une utilisation future. Ils doivent-être déclarés, également, dans la section `## Generate message in the msg folder` du fichier `CMakeLists.txt` comme suit :
</div>

```
## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RecvString.msg
   PointM.msg
   Robot.msg
)
```

##### Répertoire SRV
---------------------
<div align="justify">
Il contient les fichiers de services qui sont (ou seront) exploités dans le package `aau_rng_multi_robot`. Pour le moment, deux services ont été déclarés. Un premier service, `ShutDown.srv`, à appeler pour arrêter le n&oelig;ud. Ce service peut-être implémenté dans n'importe quelle node du package et peut-être appelé par un node client pour arrêter le second node qui implémenté la partie serveur. Le second service, `MaxDistance.srv`, est appelé par un client implémenté dans le module de couverture des point-d-intérêts (i.e., <strong>PoI</strong>). Le serveur est implémenté au niveau du node `aau_rng_communication`. Ce service fournit la distance maximale qu'a le robot avec ces robots voisins RNG Locaux. Cette distance est exploitée par l'algorithme **PoI** à traverse la formule suivante :
</div>

$$
d \le (R - d^+(u))
$$

<div align="justify">
Cette distance maximale des robots LRNG remplacera $$$d^+(u)$$$ dans la formulation précédente. Cette formule permet de calculer la distance à parcourir à chaque étape du processus de couverture du point d'intérêt.

A l'image des messages, les services doivent être déclarés dans la section `## Generate services in the 'srv' folder` du fichier `CMakeLists.txt` comme suit :
</div>

```
## Generate services in the 'srv' folder
add_service_files(
   FILES
   ShutDown.srv
   MaxDistance.srv
)
```

##### Répertoire Launch
-----------------------
Le répertoire `launch` contient deux fichiers de lancement :
 * turtlebot_setup.launch
 * aau_rng_adhoc_communication.launch

Ils doivent-être lancés dans l'ordre indiqué ci-dessus.

<div align="justify">
Le premier fichier, `turtlebot_setup.launch`, lance à son tour le fichier `bringup ` qui démarre la base du <strong>Turtlebot2</strong>. Un fois que la base est démarrée, il lance ensuite le module de navigation assistée du robot. Ceci permet ensuite d'appeler des fonctionnalités de la navigation dans nos modules pour réaliser la couverture du point d'intérêt ou pour avoir la position du robot à tout instant dans la map de navigation. Son contenu est décrit ci-dessous :
</div>

```bash
<?xml version="1.0"?>
<launch>

   <include file="$(find turtlebot_bringup)/launch/minimal.launch"/>
   <include file="$(find turtlebot_navigation)/launch/amcl_demo.launch"/>

</launch>
```
Pour le lancer, il suffit de taper la commande suivante :

```bash
$ roslaunch aau_rng_multi_robot turtlebot_setup.launch

```
Lorsqu'il est lancé, au bout quelques seondes, vous aurez une sortie similaire à ce qui est décrit ci-dessous :

```bash
[ INFO] [1572362731.039956109]: Using plugin "static_layer"
[ INFO] [1572362731.090427892]: Stopping device RGB and Depth stream flush.
[ INFO] [1572362731.145681233]: Requesting the map...
[ INFO] [1572362731.361477886]: Resizing costmap to 736 X 608 at 0.050000 m/pix
[ INFO] [1572362731.459412221]: Received a 736 X 608 map at 0.050000 m/pix
[ INFO] [1572362731.483779780]: Using plugin "obstacle_layer"
[ INFO] [1572362731.496212049]:     Subscribed to Topics: scan bump
[ INFO] [1572362731.694108520]: Using plugin "inflation_layer"
[ INFO] [1572362731.941120975]: Using plugin "obstacle_layer"
[ INFO] [1572362731.951648804]:     Subscribed to Topics: scan bump
[ INFO] [1572362732.200737349]: Using plugin "inflation_layer"
[ INFO] [1572362732.431471178]: Created local_planner dwa_local_planner/DWAPlannerROS
[ INFO] [1572362732.444681891]: Sim period is set to 0.20
[ INFO] [1572362732.992832405]: Recovery behavior will clear layer obstacles
[ INFO] [1572362733.028737855]: Recovery behavior will clear layer obstacles
[ INFO] [1572362733.195161000]: odom received!
```

<div align="justify">
Lorsque vous avez ce message `odom received!`, cela voudrait dire que votre module de navigation est prêt. Il est à présent possible d'exécuter le second fichier de lancement dont la description suit.

Le second fichier de lancement permet de démarrer nos deux nodes et son contenu est décrit ci-dessous :
</div>

```bash
<?xml version="1.0"?>
<launch>
   
 <node pkg="aau_rng_multi_robot" type="aau_rng_communication.py" name="aau_rng_communication">
     <param name="x_docker_pose" value="-0.94" />
     <param name="y_docker_pose" value="-4.20" />
     <param name="robot_name" value="sam" />
     <param name="interface" value="wlp2s0" />	
     <param name="hello_period" value="4" />
     <param name="neighbor_timeout" value="8" /> 
 </node>

<node pkg="aau_rng_multi_robot" type="poi_coverage.py" name="poi_coverage">
    <param name="comm_range" type="double" value="20"/>
    <param name="epsilon_error" type="double" value="0.5"/>
    <param name="x_first_pose" type="double" value="-4.50"/>
    <param name="y_first_pose" type="double" value="-3.80"/>
    <param name="x_goal_pose" type="double" value="-14.25"/>
    <param name="y_goal_pose" type="double" value="3.85"/>
</node>

</launch>
```

<div align="justify">
Les paramètres décrits entre le bloc `<node...></node>` sont des entrées des modules. Par exemple le node `aau_rng_communication` a besoin de connaître l'interface réseau (e.g., `wlp2s0`) sur lequel il crée les sockets de communication, la fréquence d'envoie des messages `HELLO`, la position initiale (x, y) du robot, etc. Tandis que le node de couverture du point d'intérêt a besoin de connaître la porte radio de communication, la position (x, y) du point à couvrir, etc. ==++**Dans tout autre cas d'usage, l'utilisation de ces paramètres (i.e., `hello_period`, `neighbor_timeout` et `comm_range`) tels qu'ils sont présentés ci-dessus doit faire l'objet une étude préalable, en expérimentation, avant toute exploitation en production**++==.
</div>

Il se lance via la commande suivante :


```bash
$ roslaunch aau_rng_multi_robot aau_rng_adhoc_communication.launch

```

Elle donnera la sortie suivante:

```bash
started roslaunch server http://192.168.XX.XX:42245/

SUMMARY
========

PARAMETERS
 * /aau_rng_communication/hello_period: 4
 * /aau_rng_communication/interface: wlp2s0
 * /aau_rng_communication/neighbor_timeout: 8
 * /aau_rng_communication/robot_name: sam
 * /aau_rng_communication/x_docker_pose: -0.94
 * /aau_rng_communication/y_docker_pose: -4.2
 * /poi_coverage/comm_range: 20.0
 * /poi_coverage/epsilon_error: 0.5
 * /poi_coverage/x_first_pose: -4.5
 * /poi_coverage/x_goal_pose: -14.25
 * /poi_coverage/y_first_pose: -3.8
 * /poi_coverage/y_goal_pose: 3.85
 * /rosdistro: kinetic
 * /rosversion: 1.12.14

NODES
  /
    aau_rng_communication (aau_rng_multi_robot/aau_rng_communication.py)
    poi_coverage (aau_rng_multi_robot/poi_coverage.py)

ROS_MASTER_URI=http://localhost:11311

process[aau_rng_communication-1]: started with pid [31821]
process[poi_coverage-2]: started with pid [31822]

```

##### Répertoire Script
------------------------
<div align="justify">
Le répertoire des scripts contient l'ensemble des modules, au sens script python, qui constituent l'implémentation des nodes `aau_rng_communication.py` et `poi_coverage.py`. Leur diagramme fonctionnel est présenté  la figure ci-dessous :
</div>

<center>
   <img src="./Images/graphe.png" alt="Diagramme fonctionnel" title="Diagramme fonctionnel des deux nodes ROS" width="780" height="750"/>
</center>
<center>Diagramme fonctionnel des deux nodes du package AAU_RNG_MULTI_ROBOT</center>

<div align="justify">
Le diagramme fonctionnel des modules (i.e., nodes) est décrit par la figure ci-dessus. Cette figure montre des dépendances functionnelles qui existent entre les diffèrents modules, d'un point de vue script python, constituant la solution d'auto-organisation et de couverture de point d'intérêt. Les deux nodes sont présentés par les deux blocs orange clair. Elle montre que le module `aau_rng_communication` dépend du module de communication réseau à travers les sockets de communication, et le module de gestion de l'interface réseau représenté ici les blocs violets `broadcast_api` et `get_netifaces_addresses` respectivement, du module `robot_data` qui gère les informations sur le robot tels que le nom du robot, l'identifiant robot, la position instantané du robot dans l'environement (i.e., <em>map</em>), la liste de ces voisins directs, la liste de ces voisins RNG, et l'algorithme de construction du graphe RNG local, du module `data_structures` qui offre un certains nombre de classes et de variables afin de représenter les informations du robot, et la structure des paquets échangés sur le réseau via les messages de contôle `HELLO` et du module `utils`. Ce dernier est une librairie partagée qui regroupe un ensemble de fonctionnalités communes aux deux nodes ROS. Par exemple, il offre des fonctionnalités de sérialisation, et de désérialisation des messages envoyés sur le réseau, de calcul de distance, de verrouillage et de déverrouillage d'objets partagés, etc.  Le node `aau_rng_communication` offre des services au second node `poi_coverage` qui réalise la navigation vers la couverture du point d'intérêt via le module `map_navigation` et quelques fonctionnalités de la librairie partagée. Il est par conséquent fortement dépendant de la disponibilité du service `MaxDistance.srv` du node `aau_rng_communication`. Il ne sera démarré d'après le démarrage du node `aau_rng_communication` donc implicitement la disponibilité du service `MaxDistance.srv`.
</div>