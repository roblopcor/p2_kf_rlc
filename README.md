# Implementación del Filtro de Kalman en ROS 2

Este proyecto implementa el Filtro de Kalman para estimar estados en un sistema utilizando ROS 2. Se presentan dos modelos diferentes: uno simplificado y otro completo.

## Modelos

### Modelo Simplificado (Posición)
El modelo simplificado utiliza únicamente la posición como variable de estado. Este modelo es ideal para sistemas donde no se necesita estimar la velocidad y se prioriza la simplicidad computacional.

#### 1. Ruido bajo:

#### 2. Ruido alto en la medición (Q grande):

#### 3. Ruido alto en el proceso (R grande):

### Modelo Completo (Posición y Velocidad)
El modelo completo considera tanto la posición como la velocidad como variables de estado. Este modelo ofrece una estimación más precisa y es adecuado para sistemas dinámicos.

#### 1. Ruido bajo:

#### 2. Ruido alto en la medición (Q grande):

#### 3. Ruido alto en el proceso (R grande):
