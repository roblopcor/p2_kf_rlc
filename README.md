# Implementación del Filtro de Kalman en ROS 2

## Ejecución y explicación de los resultados

Lanzar primero la simulación:
```
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
```
### Modelo Simplificado (Posición)

El modelo simplificado utiliza únicamente la posición como variable de estado. Este modelo es ideal para sistemas donde no se necesita estimar la velocidad y se prioriza la simplicidad computacional.

- Ruido bajo:
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=low
```

- Ruido alto en la medición (Q grande):
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=high_measurement
```

- Ruido alto en el proceso (R grande):
```
ros2 run p2_kf_adr kf_estimation --ros-args -p noise_config:=high_process
```

### Modelo Completo (Posición y Velocidad)

El modelo completo considera tanto la posición como la velocidad como variables de estado. Este modelo ofrece una estimación más precisa y es adecuado para sistemas dinámicos.

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
