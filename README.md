# TCC: Robô Móvel Autônomo com ROS 2

Este repositório contém todo o código-fonte, ficheiros de configuração e de lançamento (launch files) para o projeto de TCC de um robô móvel 2D, utilizando ROS 2 Humble, um Raspberry Pi 4, RPLIDAR A1 e um Arduino.

O projeto está dividido em dois marcos principais:
1.  **Marco 1: Mapeamento (SLAM)** - Criação de um mapa 2D do ambiente.
2.  **Marco 2: Navegação Teleoperada** - Utilização do joystick virtual para navegação.
3.  **Marco 3: Navegação Autônoma** - Utilização do mapa para localização (AMCL) e planeamento de trajetória (Nav2).

## 🛠️ Hardware Utilizado
* **Computador de Bordo:** Raspberry Pi 3 
* **Microcontrolador:** Arduino (para controlo de motores e leitura de encoders/IMU)
* **LIDAR:** RPLIDAR A1
* **Sensores:** Encoders de Efeito Hall (nas rodas) e IMU (MPU-6050)
* **Atuadores:** Motores DC com Drivers BTS7960

## 💿 Software
* **SO (RPi & PC):** Ubuntu 22.04
* **Framework:** ROS 2 Humble
* **Pacotes Principais:** `cartographer_ros` (para SLAM), `robot_localization` (EKF), `nav2_bringup` (Navegação)

---

## 🚀 Como Usar este Repositório

### 1. Instalação (Clone e Build)

Este repositório foi desenhado para ser usado como um "overlay" do ROS 2.

```bash
# 1. Clone o repositório para o seu PC e RPi
git clone [https://github.com/lukeTec/tcc_robo_ros2.git](https://github.com/lukeTec/tcc_robo_ros2.git)

# 2. Mova os pacotes para o seu workspace
# (Cuidado: isto irá sobrescrever os seus pacotes existentes)
cd tcc_robo_ros2
cp -r ros2_ws/src/* ~/ros2_ws/src/
cp -r arduino/seu_codigo.ino ~/Arduino/seu_codigo/

# 3. Compile o workspace (no RPi)
cd ~/ros2_ws
rm -rf build install log
colcon build
