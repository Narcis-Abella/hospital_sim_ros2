# Hospital Simulation ROS 2

Este paquete de ROS 2 (`hospital_sim`) contiene un entorno de simulación completo de un hospital para **Ignition Gazebo (Fortress)**. Incluye un mapa detallado (`.sdf`) y todos los modelos 3D necesarios (camillas, máquinas de rayos X, mobiliario) integrados en el paquete para funcionar sin dependencias externas.

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

Para lanzar la simulación, simplemente ejecuta el archivo de lanzamiento principal. Este script se encarga automáticamente de configurar las variables de entorno (`IGN_GAZEBO_RESOURCE_PATH`) para que Gazebo encuentre los modelos 3D incluidos.

```bash
ros2 launch hospital_sim hospital.launch.py
```

## 📂 Estructura del Paquete

* **`launch/`**: Contiene `hospital.launch.py`, encargado de iniciar Ignition Gazebo y cargar el mundo.
* **`worlds/`**: Contiene `ros2_hospital_map_extended.sdf`, el archivo principal del entorno.
* **`models/`**: Contiene los activos 3D (meshes, texturas, configs).
* Nota: Todos los nombres de carpetas han sido normalizados a minúsculas para compatibilidad con Linux.



## 🐛 Solución de Problemas Comunes

**Los objetos se ven negros o no aparecen:**
Asegúrate de haber compilado y hecho `source` correctamente. El archivo `launch` inyecta la ruta de la carpeta `models` en las variables de entorno de Ignition. Si ejecutas el archivo `.sdf` manualmente sin el launcher, necesitarás exportar la variable tú mismo:

```bash
export IGN_GAZEBO_RESOURCE_PATH=$HOME/SocialTech_C/src/hospital_sim/models

```

## ⚖️ Créditos y Licencia

* Los modelos 3D médicos y mobiliario provienen originalmente del repositorio [AWS Robomaker Hospital World](https://github.com/aws-robotics/aws-robomaker-hospital-world).
* Adaptación y configuración para ROS 2 Humble realizada por Narcis Abella.

```
