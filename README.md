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
