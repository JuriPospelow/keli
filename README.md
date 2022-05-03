# Stand 03.05.2022
# Was ist gemacht:

Es gibt ein groben Gerüst mit allen wichtigen Funktionen: 
1. Kommunikation mit dem Sensor SDS011 und die Ermittlung der Daten PMs und AQI.
2. Ausgabe diese Daten durch:
		LEDs,
		USB,
		WebServer.
3. Umschalten zwischen Intensiv und Standard Mode mit dem Taster.

# Was ist noch offen:

1. Übergang von Arduino IDE zu VS Code und PlatformIO
2. Taster:
	   Interrupt nicht  benutzen,
	   Debouncer verwenden,
	   Parametrierung SleepTime einbauen
3. SDS011 Code in eine extra Datei schieben.
4. WebServer - siehe unten.

Webserver zwei Arten des Menüaufbaus: 
* mit Internet: Home, SDS Config, Graph, Current Data.
* ohne Internet: Home, WiFi Config, Current Data, CSV-Tabelle  

Webserver Seite "Current Data": Ampel Darstellung für AQI einbauen.

Webserver Seite "SDS Config": Parametrierung Sleep Time und Umschaltung zwischen Intensiv und Standard Modus.

Webserver Seiten "Graph" und "CSV-Tabelle":
Die Daten sollen aus einem Puffer gelesen werden. Damit die Anzeige zeigt nicht nur letzte gemessene Werte, sondern auch die welche vor dem Öffnen der Browser gemessen waren.

