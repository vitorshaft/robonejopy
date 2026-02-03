# Projeto Robonejo: Monitoramento Inteligente de Aviários

O **Robonejo** é uma plataforma robótica autônoma projetada para inspeção sanitária em aviários. O sistema combina navegação de precisão em terrenos irregulares com Inteligência Artificial para identificação e geolocalização de mortalidade e contagem de lotes.

## Conceito e Diferenciais

Diferente de soluções convencionais, o Robonejo opera sob o conceito de **Fusão Sensorial Total**:

* **Navegação Sem Deriva:** Combina IMU, Magnetômetro e *Optical Flow* (Visual) para manter a posição exata mesmo com patinagem na maravalha.
* **Monitoramento Ativo:** Utiliza visão computacional e sensores infravermelhos para detecção sanitária.
* **Fail-safe:** Capaz de navegar "às cegas" por bússola em caso de obstrução da câmera por poeira ou animais.

---

## Estrutura do Sistema (Nós ROS 2)

O ecossistema é composto por nós modulares que garantem alta disponibilidade:

| Nó | Responsabilidade | Hardware/Tecnologia |
| --- | --- | --- |
| `imu_node` | Orientação absoluta e compensação de inclinação. | MPU6050 + QMC5883L |
| `dead_reckoning_node` | Estimativa de pose  via Fusão Inercial-Visual. | OpenCV (Optical Flow) |
| `detectorSQLite` | Detecção de aves e persistência de dados. | YOLOv8 + SQLite |
| `temperature_node` | Leitura térmica sem contato para triagem de saúde. | MLX90614 (I2C) |
| `lane_detect_node` | Controle de direção (CTE) e manutenção de rumo. | PID Control |

---

## Módulos de Inteligência Sanitária

### Detecção e Contagem (YOLOv8 + SQLite)

O nó `detectorSQLite` processa o fluxo de vídeo para identificar e contar as aves.

* **Persistência:** Cada detecção relevante é armazenada em um banco de dados local (SQLite) com *timestamp* e coordenadas da pose do robô.
* **Contagem Real-time:** Utiliza algoritmos de rastreamento para evitar contagens duplicadas da mesma ave.

### Inspeção Térmica (MLX90614)

O nó `temperature_sensor_node` monitora a assinatura de calor do ambiente.

* **Detecção de Mortalidade:** Ao cruzar os dados de detecção visual do YOLO com a temperatura lida pelo MLX90614, o sistema identifica aves mortas (corpos frios em relação ao padrão do lote).
* **Alerta Georreferenciado:** O robô marca no mapa o local exato onde uma irregularidade térmica foi detectada.

---

## Visualização no Foxglove Studio

O projeto inclui um dashboard customizado para telemetria completa.

### Como visualizar:

1. Abra o [Foxglove Studio](https://studio.foxglove.dev/).
2. Vá em **Layout** > **Import from file**.
3. Selecione o arquivo: `config/robonejo_dashboard.json`.

**Tópicos monitorados:**

* `/robot_pose_estimated`: Trajetória em tempo real.
* `/camera/processed_image`: Bounding boxes do YOLO e erro de faixa.
* `/ambient_temperature`: Dados térmicos do MLX90614.

---

## ⚙️ Como Operar

### Configuração do Ambiente:

```bash
# Build do workspace
colcon build --symlink-install
source install/setup.bash

```

### Inicialização (Launch):

```bash
# Executa todos os nós de navegação e sensores
ros2 launch robonejopy robot_sensors.launch.py

```

---

## 🗺️ Roadmap de Desenvolvimento

* [x] Fusão Sensorial (IMU + Mag + Optical Flow).
* [x] Integração YOLOv8 com persistência SQLite.
* [x] Monitoramento Térmico via MLX90614.
* [ ] Navegação para base de recarga automática.
* [ ] Interface Web para relatórios automatizados de mortalidade.