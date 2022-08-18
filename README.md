# Keli
Messung von Feinstaub mit ESP32, SDS011.

Durch die Verwendung eines Webservers ist es möglich, die aktuellen Sensorwerte von jedem beliebigen Gerät im Netzwerk abzurufen.

|Start| Current Data | Graphs |
| ------ | ------- | ------- |
| ![hauptseite](https://user-images.githubusercontent.com/90817262/185370767-879ddc40-4589-4a10-a9f5-8c6a177b2be3.png) | ![current_data](https://user-images.githubusercontent.com/90817262/185370808-b79cdfa1-98c9-4cf4-9922-eb4261baa481.png) | ![graph](https://user-images.githubusercontent.com/90817262/185370827-6ee7954a-ca73-4e5f-929d-98884aa1f34a.png)




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

