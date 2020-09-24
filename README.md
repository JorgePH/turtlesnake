# Turtlesnake Game

El juego está hecho usando ROS Melodic sobre Ubuntu 18.04. Si tenéis algún error de compatibilidad me decís!

Para ejecutar la versión 1.0 sólo hace falta seguir los requisitos de la guía. Una vez clonado el repositorio, y habiendo hecho source sobre la instalación de ROS:

```
cd turtlesnake

% Cambiar al tag v1.0
git checkout v1.0

% Compilar los paquetes
catkin_make

% Hacer source
source devel/setup.bash
```

Ejecutar primero el .launch con:

```
roslaunch turtlesnake turtlesnake_launcher.launch
```

Esto lanza los nodos de turtlesim, e inicializa los broadcast y el turtlelistener.



Para empezar el juego, ejecutar en otra ventana y con source sobre la instalación:

```
rosservice call /start_turtlesim_snake
```

Que inicia el juego de snake

