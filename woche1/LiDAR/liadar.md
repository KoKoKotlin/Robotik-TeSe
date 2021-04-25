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

- ROS1: https://github.com/SICKAG/sick_scan
- ROS2: https://github.com/SICKAG/sick_scan2

## Ideen:

- Kalibration Kamera/LiDAR? (https://github.com/ankitdhall/lidar_camera_calibration)
- Object - Tracking (https://github.com/praveen-palanisamy/multiple-object-tracking-lidar)