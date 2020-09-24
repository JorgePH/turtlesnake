# Turtlesnake Game

El juego está hecho usando ROS Melodic sobre Ubuntu 18.04. Si tenéis algún error de compatibilidad me decís!

## Instrucciones

### V1.0

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

Que inicia el juego de snake.

### V2.0

```
% Cambiar al tag v2.0
git checkout v2.0

% Borrar las carpetas devel y build y recompilar
```

Esta versión acepta argumentos en el service *start_turtlesim_snake* de la forma [x y theta].

Sustituir la llamada al servicio anterior por:

`rosservice call /start_turtlesim_snake x y theta`

Donde *x*, *y* y *theta* son los valores de posición y rotación que se le quiere dar a la segunda tortuga.

### V3.0

La cosa se pone interesante.

Para lanzar esta versión hay que hacer lo mismo que para la v1.0:

```
% En una ventana
roslaunch turtlesnake turtlesnake_launcher.launch

% En otra distinta
rosservice call /start_turtlesim_snake
```

He cambiado el tipo de servicio porque las tortugas se generan internamente dentro del mismo nodo. Hay un único nodo *listener* que asigna las velocidades a todas las tortugas existentes. Las tortugas se generan y gestionan dentro de este nodo, a través de una clase *Turtle*.

Cada tortuga lanza un nodo nuevo con el broadcaster de su tf. El nodo *listener* recoge todos estos broadcast.

En la siguiente imagen se puede ver un ejemplo de los nodos activos y sus topics cuando hay un total de cinco tortugas.

<p align="center">
  <img src="https://github.com/JorgePH/turtlesnake/blob/master/images/rosgraph_v3.0.svg" width="800">
</p>
