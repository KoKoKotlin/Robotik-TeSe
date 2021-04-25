# LiDAR

## Metadaten:

| Name                | Value             |
| ------------------- | ----------------- |
| Produktname         | TIM551-2050001    |
| Hersteller          | SICK              |
| Lichtquelle         | 850nm (Infrarot)  |
| Scannbereich        | 270°              |
| Winkelauflösung     | 1°                |
| Funktionsreichweite | 0.05m - 10m       |
| Delay               | ~67ms             |
| Systematischer Err. | ±60mm             |
| Statischer Err.     | <20mm             |
| Interface           | TCP/IP od. USB    |
| Arbeitstemperatur   | -25°C - 50°C      |
| Montagehöhe         | TBD               |
| Scanbereichdim.     | 2D                |

* Tatsächlicher Scannbereich varriert je nach Umgebungs-/Oberflächeneigenschaften

* Umgebungslichtunabhägig
* Lagedaten in Messung einbezogen

## Libs:
### ROS1:

#### sick_scan (Treiber)

- ROS1: https://github.com/SICKAG/sick_scan
- Installation: https://github.com/SICKAG/sick_scan#installation

- Node starten: `roslaunch sick_scan sick_tim_511.launch`

- grafische Darstellung mit `rviz`

| Paramter                   | Default Value     | Beschreibung                           |
| -------------------------- | ----------------- | -------------------------------------- |
| hostname                   | 192.168.0.1       | Ip Addr für TCP Server                 |
| port                       | 2112              | Port für TCP Server                    |
| min_ang                    | -                 | Scanwinkel Beginn Scan (rad)           |
| max_ang                    | -                 | Scanwinkel Ende Scan (rad)             |
| use_binary_protocol        | binary            | Toggle Ascii Binary mode               |
| intensity                  | -                 | Intensitäts Werte übermitteln?         |
| intensity_resolution_16bit | -                 | Toggle 16bit/8bit Auflösung Intensität |
| cloud_topic                | -                 | Topicname für point clound data        |

### ROS2:

#### sick_scan2 (Treiber)

- ROS2: https://github.com/SICKAG/sick_scan2
- Installation: https://github.com/SICKAG/sick_scan2#installation

- Node starten: `ros2 launch sick_scan2 sick_lms_51.launch.py`

- grafische Darstellung mit `rviz2`

## Ideen:

- Kalibration Kamera/LiDAR? (https://github.com/ankitdhall/lidar_camera_calibration)
- Object - Tracking (https://github.com/praveen-palanisamy/multiple-object-tracking-lidar)