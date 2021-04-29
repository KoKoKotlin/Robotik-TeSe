# Bericht der Woche 22.4. - 29.4.

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

\pagebreak
## ROS und ROS2 
* Bibliothek erfüllen identische Funktionalitäten
### <u>Topics:</u>
- /cloud ([sensor_msgs/msg/PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html))
- /imu ([sensor_msgs/msg/Imu](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html)) <sup>1</sup>
- /scan ([sensor_msgs/msg/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html)) <sup>2</sup>

[1]: Internal measurment unit (Geometrie, Geschwindigkeitsinformationen)

[2]: Single Scan from Sensor (Rohdaten)

\pagebreak

# GNSS
## ROS sensor_msgs/NavSatFix Message

```
uint8 COVARIANCE_TYPE_UNKNOWN=0
uint8 COVARIANCE_TYPE_APPROXIMATED=1
uint8 COVARIANCE_TYPE_DIAGONAL_KNOWN=2
uint8 COVARIANCE_TYPE_KNOWN=3
std_msgs/Header header
sensor_msgs/NavSatStatus status
float64 latitude
float64 longitude
float64 altitude
float64[9] position_covariance
uint8 position_covariance_type
```
\pagebreak

## Mögliche Fehler
Fehler, die die Genauigkeit der Positionierung durch GPS / GNSS-Methoden beeinträchtigen hängen zusammen:

* Das Signal wird während seines Durchgangs durch die Atmosphäre auf variable Weise verlangsamt.
* Das Signal ist möglicherweise blockiert und erreicht den Empfänger in Städten aufgrund von Bäumen, Brücken, Tunneln usw. nicht.
* Das Signal kann von Elementen auf dem Boden (Metalloberflächen, Fenster, Gebäude usw.) reflektiert werden.
* Fehler, die sich auf die Entfernungsmessung zwischen den verschiedenen Satelliten und dem Empfänger des Benutzers auswirken

\pagebreak

# Odometrie Datenformat

## Husky Implementierung

* drei Topics bereits implementiert

|Topic                         | Sources               | Description                               |
|------------------------------|-----------------------|-------------------------------------------|
|husky_velocity_controller/odom| husky_node            | Raw odometry as read from Husky encoders  |
|imu/data                      | mu_filter_madgwick    | Orientation estimate from the IMU         |
| odometry/filtered            | ekf_localization_node | Fused odometry estimate (encoders and IMU)|


## nav_msgs/Odometry.msg
```
# This represents an estimate of a position and velocity in free space.  
# The pose in this message should be specified in the coordinate frame given by header.frame_id.
# The twist in this message should be specified in the coordinate frame given by the child_frame_id
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```
