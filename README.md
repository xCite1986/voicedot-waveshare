# VoiceDot Waveshare

Firmware für einen Home-Assistant-Sprachassistenten auf dem
**Waveshare ESP32-S3-AUDIO-Board** — mit lokalem Wake-Word, Aufnahme,
Assist-Pipeline und Sprachausgabe direkt auf dem Gerät.

> Auf Hardware verifiziert: ESP32-S3 rev v0.2, 8 MB PSRAM, alle vier
> I²C-Bausteine erkannt, Wake-Word aktiv, Gespräche end-to-end getestet.

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
- **Lautstärke per Sprache**: „Lautstärke 5" wird lokal ausgewertet.
- **REST-API** für Ansagen und Lautstärke, plus optionale Zustandsmeldung als
  Entität in Home Assistant.
- **Serial-Log im Webinterface** — die letzten 200 Zeilen ohne USB-Kabel.
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
- OGG/Opus wird nicht dekodiert — in HA MP3 oder WAV als TTS-Format wählen.
- HTTPS zu Home Assistant läuft mit `setInsecure()`, das Zertifikat wird nicht
  geprüft.
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
