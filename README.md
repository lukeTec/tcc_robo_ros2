# TCC: Robô Móvel Teleoperado com ROS 2

Este repositório contém todo o código-fonte, ficheiros de configuração e de lançamento (launch files) para o projeto de TCC de um robô móvel 2D, utilizando ROS 2 Humble, um Raspberry Pi 3, RPLIDAR A1 e um Arduino Due.

O projeto está dividido em dois marcos principais:
1.  **Marco 1: Mapeamento (SLAM)** - Criação de um mapa 2D do ambiente.
2.  **Marco 2: Navegação Teleoperada - Utilização do joystick virtual para navegação.

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

### Passo 1: Instalação do ROS 2 Humble
Se você já tem o ROS 2 Humble instalado no Ubuntu 22.04, pule para o Passo 2.

Abra o terminal e execute os comandos abaixo para configurar o sistema e instalar o ROS 2:

```bash
# 1. Certifique-se de que o locale suporta UTF-8
locale  # Verifique se é UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. Adicione o repositório do ROS 2
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. Instale o ROS 2 Humble e ferramentas de desenvolvimento
sudo apt update
sudo apt install ros-humble-desktop ros-dev-tools -y

# 4. Adicione o ROS ao seu ambiente (bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
### Passo 2: Instalação (Clone e Build) do workspace

Este repositório foi desenhado para ser usado como um "overlay" do ROS 2.
```bash
# 1. Clone o repositório para o seu PC e RPi
git clone -b feature/teleop-manual https://github.com/lukeTec/tcc_robo_ros2.git

# 2. Instale as dependências do projeto (Lidar, Cartographer, Nav2, etc.)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# 3. Compile o workspace (no RPi)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### Passo 3: Configuração do Arduino (Microcontrolador)
O Arduino Due precisa estar rodando o código de firmware para receber comandos do Raspberry Pi.

Instale o Arduino IDE no seu computador pessoal.

Navegue até a pasta onde você clonou o repositório (ou baixe apenas o arquivo .ino):

Conecte o Arduino Due ao PC via USB (Porta de Programação).

Selecione a placa Arduino Due (Programming Port) e faça o Upload.

Importante: Agora desconecte o Arduino do PC e conecte na USB do Raspberry Pi.

Permissões da Porta Serial (No Raspberry Pi): Para que o ROS consiga falar com o Arduino sem dar erro de "Permission Denied":
```bash
# Adiciona seu usuário ao grupo de permissão de serial
sudo usermod -a -G dialout $USER

# Reinicie o Raspberry Pi para aplicar a mudança
sudo reboot
```

🎮 Como Usar (Execução)
Certifique-se sempre de estar na pasta ~/ros2_ws e ter executado source install/setup.bash.

Modo A: Teleoperação (Controle Manual)
Para controlar o robô com o joystick e visualizar os dados dos sensores.

1. No Raspberry Pi (via SSH) - Inicie os nós base do robô (drivers e comunicação com Arduino):

Execute o comando:
```bash
ros2 launch meu_pacote_navegacao teleop_bringup.launch.py
# Nota: Substitua 'meu_pacote_navegacao' pelo nome definido no package.xml dentro de 'src'
# e 'teleop_bringup.launch.py' pelo nome do seu arquivo de launch de joystick.
```
2. No seu PC (Computador de Controle)
Agora vamos abrir o controlador virtual. Abra um terminal no seu PC (não no RPi) e execute:
```bash
# Se ainda não tiver o pacote instalado, instale-o (apenas na primeira vez):
sudo apt install ros-humble-rqt-robot-steering

# Execute a interface de controle:
ros2 run rqt_robot_steering rqt_robot_steering
```
Como configurar a janela que abriu:
Na janela do rqt, procure o campo Topic.
Digite ou selecione: /cmd_vel_teleop (substitua para o tópico que você criou)
Agora você pode usar os "sliders" (barras deslizantes) verticais e horizontais na tela para controlar a velocidade linear e angular do robô.

**Nota de Rede: Se o PC não conseguir controlar o robô, verifique se ambos estão usando o mesmo Domain ID. Adicione a linha export ROS_DOMAIN_ID=0 no final do arquivo ~/.bashrc tanto no PC quanto no Raspberry Pi.

Modo B: Mapeamento (SLAM)
Para criar um mapa do ambiente enquanto você dirige o robô.

Inicie o robô e o SLAM:
```bash
ros2 launch meu_pacote_slam iniciar_cartographer.launch.py
```
Dirija o robô pelo ambiente devagar (com a velocidade 0,1m/s ou menos) para preencher o mapa.

Quando terminar, salve o mapa:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/meu_mapa
```
Use o RViz2 pelo seu computador, para visualizar o mapa e o robô pelo seu notebook/desktop.
