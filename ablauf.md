# Organisation/Ablauf

## Organisation

* Meeting am Anfang der Woche (Aufgaben Aufteilen [im Idealfall Gruppen, um Brainstormen zu Erlauben und triftige Fehler zu vermeiden, Moral und somit die       Motivation zu steigern {Püntlichkeit, Ordentlichkeit, usw}])
* Meeting am Ende der Woche (Zusammenfassen, Zusammentragen, Verschmelzen des geschafften)
* Erledigen der Gruppenaufgaben zu geregelten Zeiten um Struktur einzubringen
* Feste Verbindlichkeiten (GitHub, ROS2 Codebase, UML, Präsentationen)
* Kommunikation via Discord
* Datenaustausch via GitHub

## Ablauf

* Definieren des Problems (brechen in kleine Portionen)
* Kommunikation mit anderen Gruppen (Datenformat, technische Eigenschaften der Sensoren)
* Recherchieren der benötigten Mittel
* Recherchieren von GitHub-Bibliotheken
* Struktogramme/Ablaufpläne/UML-Klassendiagramm erstellen
* Analysieren der GitHub-Bibliotheken
* Ansätze/Ideen der Integration der GitHub-Bibliotheken
* Schreiben der einzelnen Nodes & Eigenschaften
* Integration des großen Ganzen
* Aufspielen auf Husky

## Implementierung der einzelnen Sensoren

* grobe Implementierung aller Sensoren
* Verfeinerung (Daten filtern, Sanitychecks)

\pagebreak

## Grober Zeitplan
* 2 Wochen:
    - Definition des Problems 
    - Absprache Datenformate $\hookrightarrow$ Erstellung Datenformate ROS 
    - Recherche verbaute Sensoren
    - Sichten von Bibliotheken
    - Erstellen UML-Klassendiagramme
    - Festlegung Zugriffsverfahren auf Sensoren (pubSub, action)
* 3 Wochen:
    - Grobe Implementierung Sensoren
    - Testen der Bibliotheken
    - Testen der Funktionalität der Sensoren (so weit wie möglich)
    - Implementierung Datenformate in ROS
* 5 Wochen:
    - Implementierung von Filterungalgorithmen