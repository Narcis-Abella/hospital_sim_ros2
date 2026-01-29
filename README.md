# Hospital Simulation ROS 2

Este paquete de ROS 2 (`hospital_sim`) contiene un entorno de simulación completo de un hospital para **Ignition Gazebo (Fortress)**. Incluye un mapa detallado (`.sdf`) y todos los modelos 3D necesarios (camillas, máquinas, mobiliario) integrados en el paquete.

## 📋 Requisitos

* **OS:** Ubuntu 22.04 (Jammy)
* **ROS Version:** ROS 2 Humble Hawksbill
* **Simulador:** Ignition Gazebo Fortress
* **Paquetes adicionales:**
    * `ros-humble-ros-gz`

## 🛠️ Instalación

1.  **Clonar el repositorio** en tu workspace de ROS 2:
    ```bash
    cd ~/SocialTech_C/src
    git clone [https://github.com/Narcis-Abella/hospital_sim_ros2.git](https://github.com/Narcis-Abella/hospital_sim_ros2.git)
    ```

2.  **Instalar dependencias:**
    ```bash
    sudo apt update
    sudo apt install ros-humble-ros-gz
    ```

3.  **Compilar el paquete:**
    ```bash
    cd ~/SocialTech_C
    colcon build --symlink-install --packages-select hospital_sim
    ```

4.  **Cargar el entorno:**
    ```bash
    source install/setup.bash
    ```

## 🚀 Ejecución

Para lanzar la simulación, simplemente ejecuta el archivo de lanzamiento principal. Este script detecta automáticamente la ubicación de los modelos en tu carpeta `src` para evitar problemas de rutas en la instalación.

```bash
ros2 launch hospital_sim hospital.launch.py

```

## 📂 Estructura del Paquete

* **`launch/`**: Contiene `hospital.launch.py`, encargado de iniciar Ignition Gazebo y cargar el mundo.
* **`worlds/`**: Contiene `2026_hospital.sdf`, el archivo principal del entorno.
* **`models/`**: Contiene los activos 3D (meshes, texturas, configs) necesarios para la simulación.

## 🐛 Notas Importantes

* **Rutas de Modelos:** El archivo `launch` está configurado para buscar los modelos en `src/hospital_sim/models`. Si mueves el paquete fuera de un workspace estándar, podrías necesitar ajustar la ruta en `hospital.launch.py`.
* **Drivers Gráficos:** Es normal ver advertencias `libEGL` o `ODE Message` en la terminal al arrancar; no afectan a la simulación.

## ⚖️ Créditos

* Mapa y configuración ROS 2 realizados por Narcis Abella.
* Los modelos 3D (activos médicos y mobiliario) provienen de la colección **`SocialTech-Gazebo`**.
