# Reto del Brazo 2 — arm_broker

**Universidad ESAN · Curso de Robótica · Ciclo 26-2**

Integrantes: Fredy Alonso Toribio Sotelo · Madeline Trisha Chavez Cervantes · Walter Alexander Yucra Leyva · Nilser Maier Villalobos Sernaque

Broker de ROS 2 (Humble) que es el único punto de acceso a un brazo JetCobot. Recibe pedidos de movimiento de varios clientes por la acción `/move_arm`, los valida, los pone en cola y los ejecuta de a uno según una política (FIFO o prioridad con envejecimiento). Publica `/joint_states` (único publicador) y el estado de la cola en `/arm/queue_state` a 5 Hz.

## Autoría

La estructura del repositorio (manifiestos, `CMakeLists.txt`, `setup.py`, interfaces, publicador de `/arm/queue_state`, nodo cliente y scripts de análisis) la entrega el curso y es idéntica para todos los equipos. Lo nuestro es lo que está dentro de los bloques `IMPLEMENTAR`:

| Archivo | Qué escribimos | Ítem |
|---|---|---|
| `fk.py` | Tabla DH y cinemática directa `fk(q)` | 1 |
| `broker.py` | `goal_callback`, `handle_accepted_callback`, `_worker`, `execute_callback` | 2 |
| `politicas.py` | `FIFO` y `SegundaPolitica` (prioridad con envejecimiento) | 2 y 3 |
| `analisis/grabar.py` | Grabador de bag propio (ver "Por qué un grabador propio") | 3 |

## Contenido

```
src/
  arm_broker/               broker, políticas, FK, cliente
  arm_broker_interfaces/    acción MoveArm y mensaje QueueState
herramientas/               generar_carga.py, verificar_fk.py
analisis/                   grabar.py, exportar_csv.py, metricas.py
```

## Requisitos

- ROS 2 Humble, `colcon`, `rosbag2` con el plugin de almacenamiento por defecto y `python3-matplotlib`.
- `ROS_DOMAIN_ID` igual en todas las máquinas (en nuestro laboratorio, 62).
- El driver `sync_plan_nx` corriendo en la Jetson.

## Compilar

```bash
mkdir -p ~/rb2_ws/src && cp -r src/* ~/rb2_ws/src/
cd ~/rb2_ws && colcon build
source install/setup.bash

ros2 interface show arm_broker_interfaces/action/MoveArm
ros2 interface show arm_broker_interfaces/msg/QueueState
```

En cada terminal nueva hace falta `source ~/rb2_ws/install/setup.bash`.

## Entorno de red de nuestro laboratorio

El multicast está bloqueado entre las Raspberry, así que todas las máquinas se descubren por el servidor de descubrimiento de Fast DDS que corre en la Jetson:

```bash
export ROS_DOMAIN_ID=62
export ROS_LOCALHOST_ONLY=0
unset FASTRTPS_DEFAULT_PROFILES_FILE ROS_SUPER_CLIENT
export ROS_DISCOVERY_SERVER=172.51.1.28:11811
```

## Ejecutar

```bash
# 1. En la Jetson: el driver del brazo (reiniciarlo siempre después de encender el brazo)
ros2 run jetcobot_driver sync_plan_nx

# 2. El broker (en nuestro laboratorio, en la Pi-A)
ros2 run arm_broker broker --ros-args -p politica:=fifo
ros2 run arm_broker broker --ros-args -p politica:=prioridad -p tau_envejecimiento_s:=8.0

# 3. Un cliente por integrante o por Raspberry
ros2 run arm_broker cliente --ros-args -p client_id:=ana -p priority:=1 -p traza:=carga.csv

# 4. Ver la cola
ros2 topic echo /arm/queue_state arm_broker_interfaces/msg/QueueState
```

### Parámetros del broker

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `politica` | `fifo` | `fifo` o `prioridad` |
| `tau_envejecimiento_s` | 8.0 | En prioridad, cada τ segundos de espera suben un nivel |
| `cola_max` | 20 | Pedidos en espera antes de rechazar |
| `paso_max_rad` | 1.2 | Salto articular máximo respecto a la pose actual |
| `duracion_movimiento_s` | 3.0 | Duración de cada movimiento |
| `pasos_interpolacion` | 10 | Mensajes `/joint_states` por movimiento |

### Parámetros del cliente

| Parámetro | Por defecto | Qué hace |
|---|---|---|
| `client_id` | `alumno` | Nombre del cliente |
| `priority` | 1 | De 0 a 255; **mayor número = más urgente** |
| `traza` | (vacío) | CSV con una pose de 6 ángulos (rad) por fila |
| `repeticiones` | 1 | Veces que recorre la traza |
| `pausa_s` | 0.5 | Pausa entre pedidos |

## Cómo decide el broker

1. **Admisión** (`goal_callback`): rechaza si hay un ángulo fuera de límites, si la punta queda fuera del alcance (80 a 480 mm de la base, con z ≥ 0), si el salto articular supera `paso_max_rad` o si la cola está llena.
2. **Cola** (`handle_accepted_callback`): solo encola; no ejecuta ni publica.
3. **Trabajador** (`_worker`): un solo hilo. Saca el siguiente pedido con la política y no toma otro hasta que el actual termine (exclusión mutua).
4. **Ejecución** (`execute_callback`): interpola en `pasos_interpolacion` pasos, publica `/joint_states`, envía feedback `QUEUED` o `EXECUTING` y atiende la cancelación.

Políticas: FIFO elige el de menor hora de llegada. Prioridad con envejecimiento elige el mayor puntaje, `puntaje = prioridad + espera_s / τ`, y desempata por llegada.

## Medir (ítem 3)

```bash
python3 herramientas/generar_carga.py --n 40 --semilla 7 --salida carga.csv

# Con el servidor de descubrimiento, `ros2 bag record` graba bags vacíos; usar el grabador propio:
python3 analisis/grabar.py fifo          # arranca al llegar el primer pedido y para solo
python3 analisis/exportar_csv.py fifo --salida fifo/
python3 analisis/metricas.py fifo/queue_state.csv prioridad/queue_state.csv
```

Reiniciar el broker antes de cada corrida y usar la misma carga y la misma `duracion_movimiento_s` en ambas políticas.

### Por qué un grabador propio

Con el servidor de descubrimiento de Fast DDS, `ros2 bag record` no logra averiguar el tipo de los tópicos y escribe un bag vacío. `analisis/grabar.py` declara los tipos de antemano y escribe un bag normal que `exportar_csv.py` y `ros2 bag info` leen igual.

## Verificar la cinemática directa (ítem 1)

```bash
python3 herramientas/verificar_fk.py
```

Se corre en la Jetson con el puerto serie libre. Criterio del reto: error ≤ 10 mm. En nuestras tres poses el error medio fue 7.5 mm y el máximo 9.8 mm.

## Estado

| Ítem | Estado |
|---|---|
| 1. Cinemática directa | Hecho y verificado en el brazo |
| 2. Broker con cola y políticas | Hecho. Publicador único y totales verificados en las corridas; los registros de las pruebas de rechazo y cancelación (1 a 5) están por adjuntar |
| 3. Medición bajo contención | Solo corrida piloto, **no válida para comparar políticas** (dos Raspberry, un cliente efectivo y duraciones de movimiento distintas). Corrida con cuatro clientes pendiente |
| 4. Objetivo cartesiano | Código listo. Falta la corrida con el brazo; hay una verificación en simulación, rotulada como tal |
