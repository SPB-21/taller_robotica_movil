# mi_robot_diferencial

Robot móvil de tracción diferencial con LiDAR y cámara, modelado en URDF/Xacro y simulado en Gazebo con ROS 2. Recorre una pista propia con teleoperación por teclado.

**Autores:**sebastian palacio· **Curso:** Robótica, Ingeniería Mecatrónica, Universidad EIA 

---

## 1. Dependencias y ejecución

- ROS 2 Jazzy (o superior) y Gazebo Harmonic
- Paquetes: `ros-jazzy-ros-gz`, `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher`, `ros-jazzy-rviz2`, `ros-jazzy-teleop-twist-keyboard`

```bash
cd ~/ros2_ws/src && git clone <URL_DEL_REPO>
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select mi_robot_diferencial
source install/setup.bash

# Terminal 1: Gazebo + bridge + robot_state_publisher + spawn + odometrías + RViz2
ros2 launch mi_robot_diferencial mi_robot.launch.py

# Terminal 2: teleoperación
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Estructura del paquete

| Ruta | Función |
|---|---|
| `urdf/mi_robot.urdf.xacro` | Modelo del robot, plugins DiffDrive y JointStatePublisher, sensores |
| `worlds/pista.sdf` | Mundo con la pista, muros con colisión y punto de spawn |
| `config/bridge.yaml` | Puente de topics ROS 2 ↔ Gazebo |
| `config/mi_robot.rviz` | Configuración de RViz2 |
| `launch/mi_robot.launch.py` | Levanta todo con un solo comando |
| `scripts/odometria_propia.py` | Cinemática de las llantas: integra la pose desde `/joint_states` |
| `scripts/comparar_odometrias.py` | Compara `/odom` (plugin) contra `/odom_propia` y reporta el error |

---

## 3. Modelo del robot

Todo está parametrizado con propiedades Xacro: `rueda_radio`, `rueda_ancho`, `rueda_masa`, `separacion_ruedas`, `caster_radio`.

| Parámetro | Valor | Justificación |
|---|---|---|
| Chasis | caja 0.30 × 0.20 × 0.10 m, 1 kg | Tamaño de robot de laboratorio, con espacio para sensores. |
| Radio de rueda `r` | 0.05 m | Rueda comercial pequeña, con altura libre suficiente para el chasis. |
| Ancho de rueda | 0.03 m | Proporción típica 0.3·r…0.6·r. |
| Separación `L` | 0.25 m | Las ruedas quedan justo fuera del chasis (semiancho 0.10 m; cara interna de la rueda en 0.11 m). Es el mismo valor que usa el plugin y el nodo de odometría. |
| Eje de ruedas | z = −0.02 m | El fondo de la rueda queda en −0.07 m y el chasis (fondo en −0.05 m) tiene 2 cm de holgura al piso. |
| Casters (2) | esferas de 0.02 m en x = ±0.12 m, fricción 0 | El eje de las ruedas pasa por el centro de masa, así que con un solo caster el robot cabecea al frenar. Dos casters sin fricción lo estabilizan sin afectar la cinemática. |

**Inercias.** Se calculan con las fórmulas de sólidos rígidos (cilindro, caja y esfera), no con valores arbitrarios:

- Caja: `Ixx = m(b²+c²)/12` y análogos.
- Cilindro: `Izz = ½·m·r²`, `Ixx = Iyy = m(3r²+h²)/12`.
- Esfera: `I = 2/5·m·r²`.

**Giro de las ruedas.** La junta usa `rpy = (−π/2, 0, 0)` con eje z, de modo que el eje queda en +y. Así un giro positivo de la rueda hace avanzar el robot hacia +x, como asume el plugin.

---

## 4. Tracción diferencial

El plugin `gz-sim-diff-drive-system` recibe `/cmd_vel` y publica la odometría y el TF `odom → base_link`. Se configura con los mismos parámetros del modelo: `wheel_separation = 0.25`, `wheel_radius = 0.05`, odometría a 30 Hz.

---

## 5. Cinemática de las llantas

Con `w = [ωD, ωI]ᵀ` (velocidades angulares de las ruedas derecha e izquierda), `u = [v, ω]ᵀ` y `q̇ = [ẋ, ẏ, θ̇]ᵀ`:

**De w a u** (cinemática directa del diferencial):

```
u = [ v ]  =  [ r/2    r/2 ] [ ωD ]
    [ ω ]     [ r/L   −r/L ] [ ωI ]
```

**De u a q̇** (rotación al marco fijo):

```
q̇ = [ ẋ ]  =  [ cosθ  0 ] [ v ]
     [ ẏ ]     [ sinθ  0 ] [ ω ]
     [ θ̇ ]     [  0    1 ]
```

**Matriz completa de w a q̇:**

```
q̇ = [ (r/2)cosθ   (r/2)cosθ ] [ ωD ]
     [ (r/2)sinθ   (r/2)sinθ ] [ ωI ]
     [   r/L        −r/L     ]
```

Con `r = 0.05` y `L = 0.25`: `r/2 = 0.025` y `r/L = 0.2`.

**Inversa (lo que hace el plugin al recibir `/cmd_vel`):** `ωD = (v + ω·L/2)/r` y `ωI = (v − ω·L/2)/r`.

**Integración.** `odometria_propia.py` lee las velocidades de `joint_rueda_der` y `joint_rueda_izq` en `/joint_states`, calcula `v` y `ω`, e integra por Euler: `x += v·cosθ·dt`, `y += v·sinθ·dt`, `θ += ω·dt`. Publica en `/odom_propia`.

**Comparación.** `comparar_odometrias.py` calcula en cada mensaje de `/odom_propia`, contra la última pose de `/odom`:

- error de posición: `√(Δx² + Δy²)`
- error de yaw, normalizado a [−π, π]

Imprime ambos cada 2 s junto con el RMSE, publica `/error_odometria` y guarda el CSV en `/tmp/error_odometria.csv`.

### Resultados (completar tras el ensayo)

| Prueba | Error final de posición (m) | Error final de yaw (°) | RMSE pos (m) |
|---|---|---|---|
| Línea recta (~3 m) | | | |
| Giro en sitio (360°) | | | |
| Pista completa | | | |

### Por qué difieren las dos odometrías

- **Integración:** el plugin usa el incremento de posición de las ruedas, y nuestro nodo integra velocidades con Euler, que acumula error de truncamiento sobre todo en giros.
- **Tiempo:** nuestro nodo mide `dt` con el reloj de pared. Si el factor de tiempo real baja de 1, la pose se desvía.
- **Muestreo:** `/joint_states` y `/odom` llegan a frecuencias distintas y la comparación usa el último dato recibido.
- **Deslizamiento:** ninguna de las dos lo modela, y ambas se calculan desde las ruedas.

---

## 6. LiDAR

Sensor `gpu_lidar` en `lidar_link`, ubicado en x = 0.10 m y z = 0.08 m. Queda 3 cm por encima del chasis para ver 360° sin que el chasis lo obstruya.

| Parámetro | Valor | Justificación |
|---|---|---|
| FOV | 360° | Percepción completa del entorno, como los LiDAR 2D comerciales. |
| Muestras | 360 (1°) | A 8 m el espaciado es de 0.14 m, y un obstáculo de 0.3 m recibe al menos 2 rayos. |
| Rango | 0.15–8 m | El mínimo coincide con un RPLIDAR A1 y el máximo cubre el tramo recto más largo de la pista (~6 m). |
| Frecuencia | 10 Hz | Típica de LiDAR 2D de bajo costo. |
| Ruido | gaussiano, σ = 0.01 m | Realismo de sensor. |
| Frame | `lidar_link` | Se fija con `gz_frame_id` para que coincida con el árbol TF. |

## 7. Cámara

Sensor `camera` en `camera_link`, en la cara frontal del chasis.

| Parámetro | Valor | Justificación |
|---|---|---|
| Resolución | 640 × 480 | Suficiente para ver los obstáculos con poco costo de render. |
| FPS | 30 | Estándar de video. |
| FOV horizontal | 60° (1.047 rad) | Similar a cámaras de bajo costo. Da `fx = 320/tan(30°) ≈ 554 px`. |
| Clip | 0.1–10 m | La pista cabe en ese rango. |
| Ruido | gaussiano, σ = 0.007 | Realismo. |
| Frame | `camera_optical_link` | Marco óptico con x derecha, y abajo y z adelante (REP-103), rotado `rpy = (−π/2, 0, −π/2)` respecto a `camera_link`. |

---

## 8. Frames (árbol TF)

```
odom ──(plugin DiffDrive, vía /tf)──> base_link
                                        ├── lidar_link
                                        ├── camera_link ── camera_optical_link
                                        ├── rueda_izq, rueda_der   (robot_state_publisher con /joint_states)
                                        └── caster_frontal, caster_trasero
```

- `odom → base_link` lo publica Gazebo y llega por el bridge. Es la pose estimada por la odometría del plugin.
- El resto lo publica `robot_state_publisher` a partir del URDF (juntas fijas) y de `/joint_states` (ruedas).
- El fixed frame de RViz es `odom`. No hay `map` porque no se hace SLAM ni navegación.

## 9. Bridge (`bridge.yaml`)

| Topic ROS 2 | Sentido | Función |
|---|---|---|
| `/cmd_vel` | ROS → Gz | Comandos de velocidad del teleop hacia el plugin. |
| `/odom` | Gz → ROS | Odometría del plugin DiffDrive. |
| `/tf` | Gz → ROS | Transformada `odom → base_link`. |
| `/joint_states` | Gz → ROS | Posición y velocidad de las ruedas, usada por el nodo de odometría y por `robot_state_publisher`. |
| `/scan` | Gz → ROS | Datos del LiDAR. |
| `/image_raw` | Gz → ROS | Imagen de la cámara. |
| `/camera_info` | Gz → ROS | Parámetros intrínsecos de la cámara. |
| `/clock` | Gz → ROS | Reloj de simulación, necesario para `use_sim_time`. |

---

## 10. Pista

Mundo `pista.sdf` de diseño propio. Todos los muros son estáticos con colisión. El spawn está en x = 0.5, y = 0, yaw = 0, marcado con un cuadro verde sin colisión.

| Tramo | Descripción |
|---|---|
| A | Pasillo recto de 1.9 m de ancho, con un **estrechamiento** de 0.8 m de paso libre (≈ 2.9 veces el ancho del robot con ruedas, 0.28 m) y el obstáculo 1 (caja 0.3 m). |
| Curva 1 | Giro a la izquierda, con la esquina cerrada para que el robot no salga de la pista. |
| B | Pasillo vertical de 1.9 m con el obstáculo 2 (caja 0.3 m) en el centro. |
| Curva 2 | Segundo giro a la izquierda. |
| C | Pasillo hacia el oeste con el obstáculo 3 (cilindro de 0.4 m de diámetro) y muro final. |

El ancho de 1.9 m deja margen para maniobrar en teleoperación, y el estrechamiento exige precisión.

---

## 11. Integración

Un solo `ros2 launch` levanta:

1. Gazebo con el mundo, sin pausa (`-r`).
2. `robot_state_publisher`, con `robot_description` generado desde el xacro.
3. El bridge.
4. El spawn del robot, con retardo de 4 s para esperar a Gazebo.
5. `odometria_propia` y `comparar_odometrias`.
6. RViz2 con la configuración guardada.

Todos los nodos usan `use_sim_time` para compartir el reloj de Gazebo.

## 12. Verificación rápida

```bash
ros2 topic echo /scan --field header.frame_id --once        # lidar_link
ros2 topic echo /image_raw --field header.frame_id --once   # camera_optical_link
ros2 topic echo /joint_states --once                        # incluye velocity
ros2 run tf2_tools view_frames                              # árbol de frames
```

