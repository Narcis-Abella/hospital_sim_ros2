# Hospital Simulation ROS 2 (Jetson Optimized)

Este paquete de ROS 2 (`hospital_sim`) contiene un entorno de simulación de un hospital y un robot móvil (Tracer) migrados a **ROS 2 Humble**.

El proyecto ha sido optimizado específicamente para ejecutarse en hardware **NVIDIA Jetson (ARM64)**, incluyendo modos de bajo consumo (Headless) y mapas optimizados (Lite).

## 📋 Requisitos

* **Hardware:** NVIDIA Jetson Orin / Xavier / Nano (Arquitectura ARM64).
* **OS:** Ubuntu 22.04 (Jammy Jellyfish).
* **ROS Version:** ROS 2 Humble Hawksbill.
* **Simulador:** Ignition Gazebo Fortress.
* **Dependencias:**
    * `ros-humble-ros-gz`
    * `ros-humble-xacro`
    * `ros-humble-joint-state-publisher-gui`
    * `ros-humble-teleop-twist-keyboard`

## 🛠️ Instalación

1.  **Clonar el repositorio** en tu workspace (`src`):
    ```bash
    cd ~/SocialTech_C/src
    git clone [https://github.com/Narcis-Abella/hospital_sim_ros2.git](https://github.com/Narcis-Abella/hospital_sim_ros2.git)
    ```

2.  **Instalar dependencias del sistema:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-ros-gz ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-teleop-twist-keyboard
    ```

3.  **Compilar:**
    *Nota: Es importante usar symlink-install para desarrollo.*
    ```bash
    cd ~/SocialTech_C
    colcon build --symlink-install --packages-select hospital_sim
    ```

4.  **Cargar el entorno:**
    ```bash
    source install/setup.bash
    ```

## 🚀 Ejecución

### 1. Lanzamiento Estándar (Modo Lite + Headless)
Por defecto, el sistema está configurado para **rendimiento máximo en Jetson**. Esto carga el mapa ligero (solo paredes) y ejecuta Gazebo en segundo plano (sin ventana 3D), visualizando todo en RViz.

```bash
ros2 launch hospital_sim hospital.launch.py

```

* **Lo que verás:** Se abrirá RViz con el robot, el mapa y los datos del láser.
* **Rendimiento esperado:** RTF > 60% en Jetson Orin.

### 2. Control del Robot (Teleoperación)

Para mover el robot, abre una **nueva terminal** y ejecuta:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard

```

* Usa las teclas `i`, `j`, `l`, `,` para moverte.
* Usa `q` / `z` para ajustar la velocidad máxima.

---

## 🔧 Configuración Avanzada

### Cambiar entre Mapa Completo vs Lite

Para cambiar el mapa cargado, edita `launch/hospital.launch.py`:

* **Modo Lite (Rápido):** `world_file = ... '2026_hospital_lite.sdf'`
* **Modo Completo (Lento):** `world_file = ... '2026_hospital.sdf'`

### Activar/Desactivar GUI de Gazebo

Si quieres ver la simulación 3D (ojo, consume mucha GPU), edita `launch/hospital.launch.py`:

* **Con GUI:** Elimina el argumento `-s` en `gz_args`.
* **Headless (Sin GUI):** Mantén el argumento `-s` (Server Only).

## 🐛 Solución de Problemas (Troubleshooting)

### Pantalla Negra en Gazebo

Si usas el sistema en español, Gazebo puede fallar al leer los decimales (punto vs coma).
**Solución:** Ejecuta esto antes de lanzar:

```bash
export LC_NUMERIC=C

```

### El robot va muy lento (RTF bajo)

1. Asegúrate de usar el mapa **Lite**.
2. Verifica que el Lidar tenga `<visualize>false</visualize>` en el URDF.
3. Lanza en modo **Headless** (solo RViz).

### Errores "libEGL" o "nvidia-drm"

Son advertencias normales en Jetson debido a los drivers compartidos de Tegra. Si RViz funciona y ves el robot, puedes ignorarlos.

## 📂 Estructura del Paquete

* **`launch/`**: Launch unificado (Gazebo + Robot + Bridge + RViz).
* **`worlds/`**:
* `2026_hospital.sdf`: Mapa completo con mobiliario (High CPU/GPU).
* `2026_hospital_lite.sdf`: Mapa optimizado solo paredes (Recomendado).


* **`urdf/`**: Archivos Xacro del robot Tracer (sensores optimizados).
* **`models/`**: Meshes y texturas.

## ⚖️ Créditos

* Migración a ROS 2 y Optimización Jetson realizada por Narcis Abella.
* Activos 3D de la colección `SocialTech-Gazebo`.

```

```
