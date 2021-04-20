# Stand der Technik

## LiDAR Sensor 

* 2D LiDAR Sensor TiM551 <br>
![LiDAR](/Lidar/LidarTim551Drawing.png)
* ROS package: https://github.com/SICKAG/sick_scan
* ROS2 package: https://github.com/SICKAG/sick_scan2

### Ausgabebeispiel:
![Example](/Lidar/Lidar-OutputExample.png)
* drei Echos des Lasers
* zwischen -45° und 225°
* sendet Distanz in Gradintervallen (im Beispiel je 0.25° eine Entfernung)
* Reichweite bis max. 10m (siehe DataSheet)
* besser bis 8m bei 10% Remission der Oberflächen

## GNSS
![GNSS](/GNSS/uBlox8.jpg)
* arbeitet mit den Galileo Satelliten (europäisches Satellitenprogramm)
* bis zu 72 Satelliten gleichzeitig
* Treiber für Linux und Windows verfügbar (keine ROS packages)
* C#/C++/VB unterstützt für Entwicklung und Integration
