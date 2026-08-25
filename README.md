# VoiceDot Waveshare

Firmware für einen Home-Assistant-Sprachassistenten auf dem
**Waveshare ESP32-S3-AUDIO-Board** — mit lokalem Wake-Word, Aufnahme,
Assist-Pipeline und Sprachausgabe direkt auf dem Gerät.

https://www.waveshare.com/esp32-s3-audio-board.htm

https://www.berrybase.at/waveshare-esp32-s3-ai-smart-speaker-development-board-dual-mikrofon-wifi-bt5-16mb-flash-240-mhz

Der Ordner `VoiceDot_Waveshare/` enthält immer den **aktuellen, lauffähigen
Stand**. Ältere Versionen stehen unter
[Releases](../../releases) und in der Git-Historie.

---

## Hardware

| Teil | Wert |
|---|---|
| Board | Waveshare ESP32-S3-AUDIO-Board |
| MCU | ESP32-S3R8, 16 MB Flash, 8 MB PSRAM |
| Audio-Codec | ES8311 · I²C 0x18 |
| Mikrofon-ADC | ES7210 · I²C 0x40 (2 Mikrofone) |
| I/O-Expander | TCA9555 · I²C 0x20 (Verstärker + Tasten K1–K3) |
| RTC | PCF85063 · I²C 0x51 |
| LEDs | 7× WS2812 Ring, **NEO_RGB**-Reihenfolge (nicht GRB) |

Die Firmware erkennt die Board-Revision beim Start selbst: sie probt beide
bekannten I²C-Pinbelegungen und wählt die mit den meisten gefundenen
Bausteinen.

---

## Funktionsumfang

- **Lokales Wake-Word** über Espressif WakeNet — Alexa, Jarvis oder Computer,
  umschaltbar im Webinterface. Kein Audio verlässt das Gerät vor der Erkennung.
- **Sprachaufnahme mit VAD**: Pre-Roll-Puffer, dauerhaft nachgeführter
  Rauschboden, Satzende über den Pegel des Sprechers statt einer festen
  Schwelle.
- **Home Assistant Assist** über WebSocket, mit `conversation_id` und
  automatischer Fortsetzung bei Rückfragen.
- **Sprachausgabe** von MP3 und WAV, mit einstellbarem Sprechtempo.
- **Ansage nach dem Wake-Word** — zufällig aus mehreren Phrasen, einmalig über
  die eigene HA-TTS erzeugt und lokal gespeichert.
- **Markdown-Bereinigung**: Antworten von Sprachmodellen werden vor der Ausgabe
  von `**`, Aufzählungszeichen und Co. befreit.
- **Tag/Nacht-Profil** für Lautstärke und LED-Helligkeit, per NTP und RTC.
- **Lautstärke per Sprache**: „Lautstärke 5", „leiser" und „lauter" werden
  lokal ausgewertet.
- **Lautstärke nach Umgebungslärm**: läuft der Fön, wird die Antwort lauter.
- **REST-API** für Ansagen und Lautstärke, plus optionale Zustandsmeldung als
  Entität in Home Assistant.
- **Updates über das Netz**: der VoiceDot holt sich neue Firmware selbst aus
  den GitHub-Releases, ausgewählt im Webinterface oder in Home Assistant.
- **Serial-Log im Webinterface** — die letzten 200 Zeilen ohne USB-Kabel.
- **Webradio** aus einer eigenen Senderliste, gestartet per Sprache
  („Spiele Energy Wien") und beendet mit „Stopp".
- **Mehrere VoiceDots** handeln untereinander aus, wer antwortet — der mit dem
  lauteren Signal führt das Gespräch.
- **mDNS-Dienst** `_voicedot._tcp` für automatische Erkennung.

---

## Build

Abhängigkeiten:

- Arduino ESP32 Core 3.x
- Bibliothek **Adafruit NeoPixel**
- **esp-sr** — fest im ESP32-Core gelinkt, nichts zu installieren.
  Die Wrapper-Library `ESP_SR` wird bewusst *nicht* verwendet.
- `mp3_decoder.cpp` / `mp3_decoder.h` — liegen im Sketch-Ordner

Board-Einstellungen in der Arduino IDE:

| Einstellung | Wert |
|---|---|
| Board | ESP32S3 Dev Module |
| Flash Size | 16MB |
| PSRAM | OPI PSRAM |
| USB CDC On Boot | Enabled |
| **Partition Scheme** | **Custom** |

„Custom" nimmt die `partitions.csv` aus dem Sketch-Ordner — sie enthält neben
zwei OTA-Slots eine eigene `model`-Partition ab `0xC10000` für die
WakeNet-Modelle. Bewusst **nicht** das fertige Schema „ESP SR 16M": das würde
bei jedem Upload Espressifs Standard-Blob dorthin schreiben und das eigene
überbügeln.

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=custom,CDCOnBoot=cdc" \
  VoiceDot_Waveshare
```

### srmodels.bin erzeugen und flashen

Die Wake-Word-Modelle liegen **nicht** im Repository (Espressifs Binärdaten
werden hier nicht weiterverteilt). Sie lassen sich reproduzierbar erzeugen,
ohne kompletten ESP-IDF-Build:

1. Der Arduino-Core 3.3.8 entspricht **esp-sr 2.4.1** — der Modellsatz ist
   identisch (61 Einträge; 2.4.2 hat drei mehr, 2.3.1 weicht ab).
2. Aus dem Release-ZIP von esp-sr 2.4.1 nur diese Ordner entnehmen:
   `model/wakenet_model/wn9_alexa`, `wn9_jarvis_tts`, `wn9_computer_tts`
   und `model/vadnet_model/vadnet1_medium`.
3. Mit dem `model/pack_model.py` aus demselben Release packen:

```bash
python pack_model.py -m models_src -o srmodels.bin
```

4. Einmalig flashen — überlebt danach jeden Firmware-Upload und jedes OTA:

```bash
esptool --chip esp32s3 --port COM4 write-flash 0xC10000 srmodels.bin
```

**`vadnet1_medium` muss mit ins Blob.** Die AFE referenziert
`esp_vadn_handle_from_name`, und der Core ist mit
`CONFIG_SR_VADN_VADNET1_MEDIUM=y` gebaut — ohne das Modell scheitert die
AFE-Erzeugung. MultiNet (`mn7_en`, 2,7 MB) fehlt dagegen bewusst: diese
Firmware nutzt keine Kommandoerkennung, das spart Platz und PSRAM.

---

## Erste Inbetriebnahme

1. Flashen und starten. Ohne gespeichertes WLAN öffnet VoiceDot einen
   Setup-Accesspoint `VoiceDot-XXXXXX`, Passwort `voicedot`, Web-UI unter
   `http://192.168.4.1` (Captive Portal).
2. WLAN eintragen, speichern — das Board startet neu.
3. Danach erreichbar unter `http://<gerätename>.local`.
4. Home-Assistant-URL und **Long-Lived Access Token** eintragen
   (HA → Profil → Sicherheit) und *Verbindung testen*.
5. *Pipelines aus HA laden* und die gewünschte Assist-Pipeline wählen.
6. Einmal eine Frage stellen — dabei lernt VoiceDot Engine, Sprache und Stimme
   der Pipeline.
7. *Ansagen erzeugen* drücken, damit die Bestätigungen lokal vorliegen.

---

## Bedienung

| Eingabe | Funktion |
|---|---|
| gewähltes Stichwort | Assist-Runde starten |
| K1 | Lautstärke + |
| K2 kurz | Assist-Runde starten |
| K2 lang (>850 ms) | Stummschaltung umschalten |
| K3 | Lautstärke − |
| BOOT | LED-Helligkeit durchschalten |

LED-Ring:

| Anzeige | Bedeutung |
|---|---|
| eine grüne LED | bereit, hört auf das Stichwort |
| orange (statisch) | Setup-AP aktiv |
| pulsierend, Helligkeit folgt der Stimme | hört zu / nimmt auf |
| orange rotierend | Home Assistant denkt nach |
| pulsierend (Farbe wählbar) | Antwort wird abgespielt |
| rot | Fehler in der letzten Runde |

Farben für Zuhören und Sprechen sind im Webinterface frei wählbar.

---

## REST-API

| Methode | Pfad | Zweck |
|---|---|---|
| GET | `/api/status` | Vollständiger Status |
| GET | `/api/config` | Konfiguration (ohne Token) |
| POST | `/api/config` | Konfiguration speichern |
| POST | `/api/runtime` | Lautstärke / Helligkeit / Farben sofort setzen |
| GET/POST | `/api/announce` | Text sprechen |
| GET/POST | `/api/volume` | Lautstärke setzen |
| GET | `/api/log?since=N` | Serial-Log ab Zeile N |
| POST | `/api/ha/pipelines` | Assist-Pipelines aus HA abrufen |
| POST | `/api/ack/build` | Ansage-Clips erzeugen |
| POST | `/api/ack/test` | Zufällige Ansage abspielen |
| GET | `/api/wifi/scan` | WLAN-Scan |
| POST | `/api/ha/test` | HA-Erreichbarkeit prüfen |
| POST | `/api/audio/speaker-test` | Testton |
| POST | `/api/assist/wake` | Assist-Runde starten |
| POST | `/api/system/reboot` | Neustart |
| GET | `/api/sound/list` | Hochgeladene Klänge auflisten |
| POST | `/api/sound/upload` | Klang hochladen (multipart, Feld `sound`) |
| GET/POST | `/api/sound/delete?name=…` | Klang löschen |
| GET/POST | `/api/sound/play?name=…` | Klang abspielen |
| POST | `/api/system/factory-reset` | NVS löschen |
| POST | `/api/ota` | Firmware-Upload |

Ansagen gehen auch direkt über die Adresszeile:

```text
http://voicedot.local/api/announce?text=Hallo
```

In Home Assistant als `rest_command`:

```yaml
rest_command:
  voicedot_ansage:
    url: "http://voicedot.local/api/announce"
    method: POST
    content_type: "application/json; charset=utf-8"
    payload: '{"text": "{{ text }}"}'
```


---

## Klänge

Kleine MP3- oder WAV-Dateien lassen sich im Webinterface hochladen (max. 512 kB,
20 Stück) und in einer Ansage voranstellen:

```text
http://voicedot.local/api/announce?text=[dingdong.mp3] Es hat geläutet.
```

Mehrere Klänge hintereinander sind erlaubt, der Text danach ist optional —
`[dingdong.mp3]` allein spielt nur den Klang und spart den TTS-Aufruf.

---

## Firmware-Updates über das Netz

Nach der Ersteinrichtung braucht es kein USB-Kabel mehr. Der VoiceDot fragt die
**Releases dieses Repositories** ab, zeigt sie im Webinterface und in Home
Assistant an, und lädt die ausgewählte Version selbst herunter.

```text
POST /api/update/check              Releases von GitHub holen
POST /api/update/install?tag=v0.8.0 diese Version installieren
POST /api/update/install            die neueste installieren
```

In `/api/status` steht unter `hardware.reset_reason`, **warum** das Gerät
zuletzt gestartet ist — Einschalten, Neustart per Software, Absturz, Watchdog,
Unterspannung oder USB-Reset. Ohne das sieht ein Absturz von außen genauso aus
wie ein gewollter Neustart, und das Raten danach hat schon genug Zeit gekostet.

Der Zustand steht in `/api/status` unter `update`: installierte Version,
neueste verfügbare, Fortschritt in Prozent und die Liste der Releases.

### Wie es abläuft

Geschrieben wird immer in die **gerade nicht laufende** der beiden 3-MB-
App-Partitionen. Bricht der Download ab oder passt etwas nicht, bleibt die
laufende Version unangetastet — ein misslungenes Update kann das Gerät nicht
unbrauchbar machen.

Geprüft wird vor dem Schreiben, ob die Datei mit `0xE9` beginnt. Ein
Fehlerseiten-HTML, das mit Status 200 ankommt, würde sonst in die zweite
Partition geschrieben.

Der eigentliche Vorgang läuft aus der Hauptschleife, nicht aus dem
Request-Handler: er dauert eine halbe Minute und endet mit einem Neustart, und
solange soll kein Browser auf einer offenen Verbindung warten. Der Ring füllt
sich währenddessen blau, der Fortschritt steht auch im Webinterface.

Einstellungen, Klänge und die Senderliste liegen in einer eigenen Partition und
überleben das Update. Die **Wake-Word-Modelle ebenfalls** — die werden nicht
mitgeliefert. Braucht eine künftige Version andere Modelle, reicht ein
Netz-Update nicht und `srmodels.bin` muss einmal per USB nachgezogen werden.

### Selbst nachsehen

Alle zwölf Stunden fragt das Gerät von sich aus nach, abschaltbar im
Webinterface. Das Ergebnis wird zwischengespeichert, und alles andere — auch
die Home-Assistant-Integration — liest diesen Zwischenspeicher statt selbst bei
GitHub anzufragen: unangemeldet erlaubt die API 60 Anfragen pro Stunde und
Adresse, was eine alle zehn Sekunden pollende Integration in einer Minute
aufbrauchen würde.

### Eine Version veröffentlichen

Ein Release ohne angehängte `.bin` erscheint in der Liste, lässt sich aber nicht
installieren — das Gerät sagt das auch so. Zum Veröffentlichen gehört also:

```bash
gh release create v0.8.0 build/VoiceDot_Waveshare.ino.bin --title "v0.8.0" --notes "..."
```

---

## Lautstärke

```text
„Lautstärke 5"   setzt auf 50 %
„leiser"         eine Stufe herunter
„lauter"         eine Stufe herauf
```

Die Stufe ist im Webinterface einstellbar, 5 bis 25 Prozentpunkte,
voreingestellt 10. Alle drei Befehle wertet das Gerät selbst aus, ohne Umweg
über den Assistenten — und nur bei kurzen Äußerungen: „mach die Musik im
Wohnzimmer leiser" gehört Home Assistant, nicht diesem Lautsprecher.

### Der Deckel bei 80 %

**Was die Oberfläche 100 % nennt, sind 80 % dessen, was der Codec könnte.** Das
Lautstärkeregister des ES8311 reicht von 32 bis 255; hier ist bei 209 Schluss.
Darüber wird abgeriegelt — auch die Anhebung nach Umgebungslärm kommt nicht
darüber hinaus, sie stapelt sich sonst auf einen ohnehin hohen Pegel.

---

## Lautstärke nach Umgebungslärm

Läuft der Fön oder der 3D-Drucker, geht eine Antwort in normaler Lautstärke
unter. Der ohnehin laufend gemessene Rauschboden hebt sie deshalb an:

```text
Rauschboden −74 dBFS (still)        →  keine Anhebung
Rauschboden −60 dBFS (Drucker)      →  +5 dB
Rauschboden −50 dBFS (Fön)          →  +10 dB, gedeckelt
```

Ein Dezibel Raumlärm kostet ein Dezibel Sprachverständlichkeit, also hebt die
Regel 1:1 an. Verhandelbar ist nur der Deckel — einstellbar zwischen 0 und
18 dB, voreingestellt 10 dB. Alles unterhalb von −65 dBFS gilt als still und
bekommt nichts.

Umgesetzt wird die Anhebung als Offset auf das Lautstärkeregister des ES8311,
das in Halb-Dezibel-Schritten arbeitet. Der Umweg über die Prozentskala würde
nur zweimal quantisieren.

**Zwei Dinge, die sonst schiefgehen würden:**

- Gemessen wird ausschließlich, solange der eigene Lautsprecher still ist.
  Sonst misst das Gerät sich selbst und dreht sich hoch, weil es laut ist.
  Nachgemessen: während das Radio spielt, klettert der Rauschboden des
  Detektors von −74 auf −63 dBFS, der für die Anhebung verwendete Wert bleibt
  bei −74,6 dBFS stehen.
- Angehoben wird nur **Sprache**, nicht Musik. Musik muss nicht verständlich
  bleiben, und ein Stream, der den Raum misst, den er gerade füllt, würde sich
  selbst hinterherlaufen.

Bei einer Grundlautstärke über etwa 91 % ist kein voller Hub mehr möglich —
das Register ist dann schon fast am Anschlag.

Zu finden im Webinterface unter **AUDIO PIPELINE**, direkt unter der
Rauschboden-Anzeige, samt Live-Anzeige der aktuellen Anhebung.

---

## Webradio

```text
„Alexa"  →  „Spiele Energy Wien"     startet den Sender
„Alexa"  →  „Stopp"                  beendet ihn wieder
```

Beides wertet das Gerät **selbst** aus, so wie „Lautstärke 5": die Senderliste
liegt lokal, ein Sprachmodell könnte den Namen ohnehin nur nachschlagen.
Gesprochener Name und gespeicherter Name werden vorher auf gemeinsamen Nenner
gebracht — klein geschrieben, Umlaute ausgeschrieben, Satzzeichen weg —, und
wenn nichts exakt passt, gewinnt der Sender mit den meisten übereinstimmenden
Wörtern.

Sender pflegst du im Webinterface unter **WEBRADIO**. Das Suchfeld fragt
**radio-browser.info** ab, und zwar aus deinem Browser heraus, nicht vom Gerät:
deren API erlaubt Cross-Origin-Anfragen, dadurch braucht der VoiceDot selbst
keinen Fremddienst. Treffer ohne MP3 werden ausgeblendet, weil nur MP3
dekodiert wird.

### HTTPS-Sender werden ohne TLS versucht

Eine TLS-Verbindung kostet auf diesem Chip rund **43 kB internen RAM** —
gemessen: 52 kB frei davor, 8,8 kB danach. Das Radio spielt dann zwar weiter,
aber das Webinterface braucht für jede Seite 25 Sekunden und wirkt tot.

Die meisten Icecast-Server antworten aber auch auf Port 80. Ein `https://`-
Sender wird deshalb **zuerst ohne TLS versucht** und fällt nur zurück, wenn das
wirklich nicht geht. Ö3 und TechnoBase.FM etwa sind bei radio-browser.info nur
mit `https://` eingetragen, laufen hier aber ohne.

Klappt das nicht, wird vor dem Handshake gerechnet statt ihn zu versuchen: ohne
ausreichenden Speicher wird der Sender in einer halben Sekunde mit klarer
Meldung abgelehnt. Der Versuch selbst hätte 60 Sekunden gedauert — und das
Webinterface währenddessen ausgehungert.

### Warum das Radio mit 16 kHz läuft

TX und RX hängen am selben I2S-Takt. Liefe die Musik mit ihren nativen 44,1
oder 48 kHz, liefe das Mikrofon mit — und die Stichworterkennung braucht exakt
16 kHz, sonst hört sie nur noch Zeitraffer. Deshalb wird der dekodierte Strom
auf 16 kHz heruntergerechnet: ein Box-Filter, das alle Eingangswerte eines
Ausgangswerts mittelt und damit gleichzeitig als Anti-Aliasing dient.

Das kostet Höhen und kauft dafür, dass **„Alexa" während der Musik überhaupt
gehört werden kann**. Am Gerät nachgemessen läuft der Detektor dabei mit voller
Rate weiter (15 Feeds/s, keine Aussetzer).

### Wenn das Stichwort fällt

Die Musik pausiert für die Frage und läuft danach weiter. Ein Livestream lässt
sich nicht anhalten, deshalb bleibt die Verbindung offen und die Bytes werden
verworfen — nachträglich aufzuholen würde nur eine Verzögerung aufbauen, die
nie wieder verschwindet. Der Ring zeigt währenddessen einen langsam wandernden
Punkt in der Zuhör-Farbe.

### API

```text
POST /api/radio/play?name=Energy%20Wien
POST /api/radio/play?url=http://...
POST /api/radio/stop
POST /api/radio/save?name=...&url=...
POST /api/radio/delete?name=...
GET  /api/radio/list
```

Der Zustand steht auch in `/api/status` unter `radio` — Sender, Abtastrate des
Streams, empfangene Kilobyte, Aussetzer und Neuverbindungen.

---

## Mehrere VoiceDots im Haus

Stehen zwei Geräte in benachbarten Räumen, hören beide dasselbe Stichwort — eines
laut, das andere leise. Ohne Absprache würden beide aufnehmen und beide denselben
Befehl an Home Assistant schicken.

Deshalb meldet jedes Gerät bei einer Erkennung per UDP-Rundruf (Port 4210), wie
laut es das Wort aufgenommen hat, und wartet kurz auf Gegenangebote. Das lauteste
Gerät führt den Dialog, alle anderen legen sich sofort wieder schlafen, ohne
überhaupt aufzunehmen.

- **Maßstab ist der Abstand zum eigenen Rauschboden**, nicht der rohe Pegel.
  Ein Gerät in einem lauten Raum gewinnt sonst nur, weil es dort lauter ist.
- **Gleichstand** (unter 0,5 dB) entscheidet die kleinere Geräte-ID — so kommen
  alle Geräte ohne Koordinator zum selben Ergebnis.
- **Wartezeit** 220 ms, einstellbar zwischen 80 und 600 ms. Steht kein zweites
  Gerät im Netz, entfällt sie ganz; ein einzelner VoiceDot zahlt also nichts
  dafür.
- Der Gewinner meldet seinen Sieg sofort, statt die anderen die volle Wartezeit
  aussitzen zu lassen.
- Die Geräte melden sich alle 30 s; nach 180 s Funkstille gilt ein Nachbar als
  weg.

Wer verliert, zeigt das auch: der Ring blinkt zweimal weich in der
Zuhör-Farbe auf und geht dann aus - sichtbares Schlafenlegen, damit man im
Raum erkennt, welches Gerät gerade übernommen hat.

Im Webinterface zeigt die Karte **Mehrere VoiceDots** die eigene ID, den eigenen
Score, die letzte Entscheidung und die bekannten Nachbarn. Abschalten lässt sich
das dort ebenfalls, und **Schlafenlegen zeigen** spielt die Animation einmal ab
(auch per `POST /api/hardware/led-test?phase=yield`).

Voraussetzung ist, dass der Subnetz-Broadcast im WLAN nicht blockiert wird —
bei getrennten VLANs oder aktivierter Client-Isolation finden sich die Geräte
nicht.

---

## Wie eine Runde abläuft

Das Audio wird **gestreamt, nicht gepuffert**: die Assist-Pipeline wird geöffnet,
bevor die Aufnahme startet, und jeder Frame geht sofort raus. Die Spracherkennung
arbeitet dadurch schon, während gesprochen wird, statt erst danach zu beginnen.
Der Verbindungsaufbau (~0,5 s) liegt hinter der Ansage und fällt nicht auf.

Der Pre-Roll-Puffer bleibt erhalten: gestreamt wird ab erkanntem Sprachbeginn,
die gepufferten 320 ms davor gehen zuerst raus. Erkennt Home Assistant mit
seiner eigenen VAD das Satzende früher als wir, wird sofort beendet.

---

## Tuning

Die VAD-Schwellen stehen als Konstanten oben in der `.ino`, die beiden
wichtigsten sind zusätzlich im Webinterface:

- **Abstand zum Störgeräusch** (3–18 dB) — muss größer sein als die
  Pegelschwankung einer laufenden Maschine im Raum, sonst gilt jede Schwankung
  als Sprache und die Aufnahme endet nie.
- **Stille bis Satzende** (0,6–3 s).

Das Satzende wird nicht am Rauschboden gemessen, sondern am **Pegel des
Sprechers**: der lauteste bisherige Frame wird mitgeführt, alles innerhalb von
`VAD_SPEECH_DROP_DB` darunter gilt weiter als Sprache. Eine feste Schwelle
knapp über dem Rauschboden schneidet sonst mitten im Satz ab, sobald jemand
Luft holt.

Der Rauschboden wird **durchgehend** nachgeführt und steht in der Karte
*Audio Pipeline*.

---

## Uhrzeit, Sommer- und Winterzeit

Die Umstellung passiert automatisch. Die Zeitzone steht als POSIX-Regel:

```text
CET-1CEST,M3.5.0,M10.5.0/3
```

Ablauf beim Start: RTC lesen → Uhr steht sofort → NTP abfragen → RTC nachziehen
(danach stündlich). Die **RTC speichert UTC**, dadurch gibt es bei der
Umstellung keine doppelte oder fehlende Stunde.

---

## Bekannte Grenzen

- Nur die eingepackten Stichworte stehen zur Wahl; weitere brauchen eine neu
  gepackte `srmodels.bin`. Ein frei erfundenes Wort trainiert Espressif nur
  kostenpflichtig.
- Kein Barge-in: während der Antwort hört der Detektor nicht zu.
- Die Aushandlung zwischen mehreren Geräten braucht Subnetz-Broadcast im selben
  Netz; über VLAN-Grenzen hinweg funktioniert sie nicht.
- OGG/Opus wird nicht dekodiert — in HA MP3 oder WAV als TTS-Format wählen.
- Sender, die **ausschließlich** über HTTPS erreichbar sind, lassen sich nicht
  abspielen — für TLS und Stichworterkennung gleichzeitig reicht der interne
  RAM nicht. Das Gerät sagt das und spielt nichts, statt unerreichbar zu werden.
- Radio nur als MP3-Stream: AAC und HLS kann der Helix-Decoder nicht. Auch
  Shoutcast-Server, die mit `ICY 200 OK` statt einer HTTP-Statuszeile
  antworten, werden abgelehnt.
- Radio klingt dumpf: 16 kHz Ausgabe sind der Preis dafür, dass das Stichwort
  während der Musik gehört werden kann.
- HTTPS zu Home Assistant läuft mit `setInsecure()`, das Zertifikat wird nicht
  geprüft.
- Firmware-Updates brauchen eine TLS-Verbindung zu GitHub, und die ist auf
  diesem Chip knapp: gemessen bleiben während des Handshakes noch rund 6 kB
  interner Heap übrig. Es funktioniert, aber viel Luft ist da nicht.
- **Die API ist unauthentifiziert** — inklusive OTA. VoiceDot gehört in ein
  vertrauenswürdiges Netz.

---

## Lizenzhinweise

`mp3_decoder.cpp` / `mp3_decoder.h` basieren laut Kopfzeile auf dem
**Helix MP3 Decoder** und wurden unverändert aus dem Ursprungsprojekt
übernommen; die dortigen Lizenzbedingungen gelten weiter.
---

## Home-Assistant-Integration

Für die komfortable Anbindung gibt es eine HACS-Integration:
[hass_voicedot-waveshare_hacs_plugin](https://github.com/xCite1986/hass_voicedot-waveshare_hacs_plugin)
— Erkennung per mDNS, ein Gerät je VoiceDot, Sensoren, Einstellungen und ein
Dienst `voicedot.announce` für Ansagen.
