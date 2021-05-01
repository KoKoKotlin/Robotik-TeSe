# LiDAR Datenfilterung

* Quelle: https://www.geo.uzh.ch/dam/jcr:609ab7d1-3cf9-4636-89a7-5d431a471dc1/msc_christian_sailer_2008.pdf

* Für sinnvolle weiterverarbeitung von Daten müssen diese zuvor gefiltert werden
    - Laserstrahl kann auch nur teilweise Reflektiert werden, was bedeutet dass mehrere Amplituden entstehen können wenn er von weiteren Hindernissen Reflektiert wird

## Elemente eines Filters:
### Datenstruktur:
    * unregelmäßig angeordnete, unverarbeitete Punktwolke aus dem R²

### Nachbarschaftsbeziehungen zwischen Punkten:
    * Punkt-Punkt-Beziehung:
        - 2 Punkte werden miteinander verglichen [wenn einer der Pkt best. Grenzwert nicht genügt -> Punkt wird als Objekt klassifiziert]
    * Punkt-Punkte-Beziehung:
        - Punkt P wird mit Punktemenge M verglichen, wenn P Kriterium dass von M definiert wird nicht genügt wird P weggefiltert
    * Punkte-Punkte-Beziehung:
        - Punktemenge A bildet Grenzwert, welche für Punktemenge B Kriterium erzeugt um B zu klassifizieren

### Filterkonzepte:
    * Steigungsmaximum:
        - siehe Punkt-Punkt-Beziehung
    * Blockminimum:
        - "[...] basiert auf horizontalem Untergrund und betrachtet, ob Pkte innerhalb oder ausserhalb einer bestimmten Pufferdistanz, dem sogenannten Blockminimum, über dem Gelände positioniert ist"
    * Oberflächenapproximation:
        - wie Blockminimum nur ohne Vorraussetzung des horizontalen Untergrund (dann kein richtiger Bock mehr, eher eine 4-eckige Schlange)
    * Clusterbildung/Segmentierung:
        - Idee: Clusterpkt gehören Objekt an und müssen daher weggefiltert werden

## im Unterschied zum Paper
    * im Paper:
        - Bodenrelief soll aufgenommen werden, dazu Objekte wie Bäume usw raus Filtern
    * wir:
        - erstes Hinderniss soll aufgenommen werden (Dinge dahinter ggf raus Filtern)