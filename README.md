<<<<<<< HEAD
# Práctica 2: Filtro de Kalman en ROS 2
Este repositorio contiene el código base para la **Práctica 2** de la asignatura de *Ampliación de Robótica*, cuyo objetivo es implementar un **Filtro de Kalman (KF)** en un entorno simulado con **ROS 2**.

El ejercicio se divide en dos partes: una primera aproximación basada en odometría con estimación de posición, y una segunda con estimación de posición y velocidad utilizando un modelo de estado extendido.

---

## Estructura del repositorio
 - kalman_filter.py # Implementación del KF con TODOs para completar 
 - kf_estimation.py # Nodo con el modelo básico de KF (posición)
 - kf_estimation_vel.py # Nodo con el modelo completo de KF (posición y velocidad) 
 - motion_models.py # Modelos de movimiento A y B 
 - observation_models.py # Modelos de observación C
 - sensor_utils.py # Funciones de alto nivel para facilitar las cosas con los sensores
 - visualization.py # Funciones de visualización de resultados
 

## Instrucciones

### Requisitos previos
Descargar el simulador y los paquetes dependientes del mismo para poder trabajar con el robot Turtlebot 4:

```bash
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-turtlebot4-simulator ros-humble-irobot-create-nodes ros-dev-tools

```

### 1. Clonar el repositorio
Fuera del docker (en tu ubuntu o en el WSL)

```bash
mkdir -p ~/AdR/p2_ws/src
cd p2_ws/src
git clone https://github.com/miggilcas/p2_kf_adr
cd p2_kf_adr
```
### 2. Construir el paquete
Ya dentro del Docker:
```bash
cd ~/AdR/p2_ws
colcon build --packages-select p2_kf_adr
source install/setup.zsh  # o setup.bash si no estás usando el docker
```
### 3. Lanzar el simulador
```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### 4. Ejecutar el nodo del filtro de Kalman
#### Modelo 1: estimación de posición
```bash
ros2 run p2_kf_adr kf_estimation
```
#### Modelo 2:
```bash
ros2 run p2_kf_adr kf_estimation_vel
```

## Objetivo de la práctica

- Comprender y programar un filtro de Kalman básico para estimar la posición del robot.
- Ampliar el modelo de estado para incluir velocidad y emplear un modelo lineal puro.
- Comparar el comportamiento del filtro con diferentes configuraciones de ruido.
- Preparar el terreno para el uso de un Filtro de Kalman Extendido (EKF) en la siguiente práctica.

## Qué deben completar los estudiantes
Los archivos kalman_filter.py, motion_models.py y observation_models.py contienen TODOs que los alumnos deben implementar.

Las clases principales son:

- KalmanFilter – Para el modelo simple (posición).
- KalmanFilter_2 – Para el modelo completo (posición + velocidad).

## Entrega
Los estudiantes deberán subir a GitHub o entregar un archivo .zip con nombre: p2_kf_<iniciales> (por ejemplo: p2_kf_mgc).

El repositorio o archivo.zip debe contener:

1. Código completo con los TODOs resueltos.

2. Capturas o gráficas de los resultados de estimación para ambos modelos.

3. Experimentos con tres configuraciones distintas:
    - Ruido bajo.
    - Ruido alto en la medida.
    - Ruido alto en el proceso.

4. Un README o una pequeña memoria en PDF explicando:
    - Cómo se ha implementado cada parte.
    - Resultados observados en los tres casos.
    - Breve análisis de por qué ocurre lo observado.

## Comentarios adicionales
Podéis cambiarle el nombre al paquete y ponerle el mismo que a la entrega, pero sed consistentes a la hora de configurar el paquete y que esté ese nombre en todos lados para que compile bien (tanto en el nombre de la carpeta donde estarán los scripts como en el setup.cfg, como en el setup.py y como en el package.xml).
=======
# Implementación del Filtro de Kalman en ROS 2

Completamos todos los scripts y las secciones TODO dentro del filtro de Kalman 1 y 2. También se ha incluido el nodo visualizador para poder ver el path real frente al path estimado por el filtro de Kalman y poder ver las gráficas en tiempo real dadas por matplotlib. Se presentan capturas en los siguientes apartados:

## Ejecución y explicación de los resultados

Lanzar primero la simulación:
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### Modelo Simplificado (Posición)

El vector de estado tiene 3 dimensiones: [x   y   thetha]

Al crear la clase KalmanFilter podemos agregar más ruido a nuestro proceso para cada una de las variables del vector de estado, es por ello que se modifican los valores de desviación para el proceso y para la observación pasándole argumentos al ejecutar nuestro nodo:

- Ruido bajo:
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=low
```
Se supone que los sensores y el modelo son muy fiables, el filtro tiene una estimación confiada y reacciona poco a los cambios bruscos:
<div align="center">
  <img src="imgs/puntos_alcanzados.png" alt="Resultado" width="500">
  </div>

- Ruido alto en la medición (Q grande):
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=high_measurement
```
No se confía en las observaciones del sensor pero sí en el del modelo. Suaviza las trayectorias ruidosas. Si el modelo tiene errores pueden presentarse desviaciones en la trayectoria, además, se tardará más en reaccionar ante cambios observados:
<div align="center">
  <img src="imgs/puntos_alcanzados.png" alt="Resultado" width="500">
  </div>

- Ruido alto en el proceso (R grande):
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=high_process
```
El filtro no confía en el modelo pero sí en las observaciones realizadas por el sensor es por ello que el filtro sigue rápidamente las mediciones. Tendrá más variabilidad si las mediciones son ruidosas:
<div align="center">
  <img src="imgs/puntos_alcanzados.png" alt="Resultado" width="500">
  </div>

### Modelo Completo (Posición y Velocidad)

El vector de estado tiene 6 dimensiones: [x   y   thetha   vx   vy   ω]

Lo mismo que en el modelo anterior, pasamos diferentes argumentos 'low', 'high_measurement' o 'high_process'.

- Ruido bajo:
```
ros2 run p2_kf_adr kf_estimation_vel --ros-args -p noise_config:=low
```

- Ruido alto en la medición (Q grande):
```
ros2 run p2_kf_adr kf_estimation_vel --ros-args -p noise_config:=high_measurement
```

- Ruido alto en el proceso (R grande):
```
ros2 run p2_kf_adr kf_estimation_vel --ros-args -p noise_config:=high_process
```
>>>>>>> origin/main
