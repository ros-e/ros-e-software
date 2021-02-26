Johannes Sommerfeldt, 2021/02

---

# Node-Tool mit Web-API für ROS-E

Das Node-Tool ist ein Backend-Programm für ROS-E mit dem Ziel, das Verwalten von ROS2-Knoten zu vereinfachen.

Wesentliche **Funktionen** des Node-Tools sind:
1. das Starten und Stoppen von Knoten
2. die Ausgabe von Log-Nachrichten jedes Knotens
3. die Einrichtung von Autostart-Konfigurationen und -Voreinstellungen
4. das Bauen einzelner Pakete im Workspace oder des gesamten Workspace

Die **Bedienung** des Node-Tools erfolgt über die Web-API, entweder direkt mit Tools wie cURL oder über das dazugehörige Frontend-Programm.  
Eine Beschreibung der API und ihrer Bedienung befindet sich im [ROS-E Wiki](https://icampusnet.th-wildau.de/ros-e/doku/-/wikis/System/Node-Tool-per-Web-API).  
<br> 

## Hinweise für Entwickler

Bei jeder Änderungen an den Endpunkten der API sollte das Node-Tool Frontend-Programm ebenfalls angepasst werden, damit die Bedienoberfläche für die API weiterhin funtioniert.  
<br> 

## Mögliche Weiterentwicklungen

Die folgenden Funktionen könnten in der zukunft noch relevant sein und in das Node-Tool und die API mit aufgenommen werden.  
<br> 

### Endpunkt für die sofortige Ausführung des Autostarts
Die Erweiterung der API um einen Endpunkt, über den alle laufenden Knoten gestoppt und die aktuelle Autostart-Konfiguration ausführt werden können, würde das Wechseln von Anwendungsfällen vereinfachen. Somit können verschiedene Entwickler abwechselnd mit der gleichen ROS-E arbeiten, ohne ROS-E vor jeder Benutzung neu starten zu müssen, um den Autostart auszuführen.  
<br> 

### Unterstützung von C++-Knoten
Das Node-Tool findet und listet bisher nur Python-Knoten von ROS 2. Falls später C++-Knoten für ROS-E programmiert wreden, sollte die Suche im Workspace so erweitert werden, dass sie auch Informationen zu Knoten in C++ findet und die entsprechenden Knoten mit auflistet.  
<br> 

### Erweiterte ROS2-Befehle
Bisher startet das Node-Tool ROS2-knoten nur über `ros2 run`-Befehle ohne Zusatzparameter. Für größere Projekte mit ROS-E könnte es sinnvoll werden, auch Launch-Konfigurationen zu starten (`ros2 launch`) oder Knoten mit zusätzlichen Parametern starten zu können, um sie zu ihrer Laufzeit in verschiedene Gruppen zu kapseln.  
Ebenfalls könnten ROS2-Befehle zur Auflistung von benutzten Topics und Parametern von ROS2-Knoten nützlich werden.

