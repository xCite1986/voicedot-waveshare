# VoiceDot Waveshare

Firmware für einen Home-Assistant-Sprachassistenten auf dem
**Waveshare ESP32-S3-AUDIO-Board** — mit lokalem Wake-Word, Aufnahme,
Assist-Pipeline und Sprachausgabe direkt auf dem Gerät.

https://www.waveshare.com/esp32-s3-audio-board.htm

https://www.berrybase.at/waveshare-esp32-s3-ai-smart-speaker-development-board-dual-mikrofon-wifi-bt5-16mb-flash-240-mhz

Der Ordner `VoiceDot_Waveshare/` enthält immer den **aktuellen, lauffähigen
Stand**. Ältere Versionen stehen unter
[Releases](../../releases) und in der Git-Historie.

**[Installationsanleitung →](INSTALL.md)** · Lizenz: GPL-3.0

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
- **Wecker mit Morning Briefing** — lokal gestellt („stelle den Wecker auf
  sieben Uhr dreißig"), Weckton frei wählbar. Das Briefing ist keine feste
  Textvorlage, sondern eine Anweisung an deine Assist-Pipeline.
- **Timer** mit gesprochener Restzeit auf Nachfrage, ebenfalls lokal.
- **Gruppensteuerung**: „schalte im Obergeschoß das Licht aus" trifft eine im
  Webinterface gepflegte Gruppe direkt, ohne Umweg über den Assistenten.
- **Mehrere VoiceDots** handeln untereinander aus, wer antwortet — der mit dem
  lauteren Signal führt das Gespräch.
- **Weboberfläche** mit Menübaum, hellem und dunklem Thema und einer
  Speicherleiste, die auf jeder Seite erreichbar bleibt.
- **mDNS-Dienst** `_voicedot._tcp` für automatische Erkennung.

---

## Installation

Schritt für Schritt in **[INSTALL.md](INSTALL.md)**: Arduino-Einstellungen,
die eigene `partitions.csv`, das Erzeugen von `srmodels.bin`, Flashen,
Ersteinrichtung und die Anbindung an Home Assistant.

Die Kurzfassung:

| Einstellung | Wert |
|---|---|
| Board | ESP32S3 Dev Module |
| Flash Size | 16MB |
| PSRAM | OPI PSRAM |
| USB CDC On Boot | Enabled |
| **Partition Scheme** | **Custom** — nimmt die `partitions.csv` aus dem Sketch-Ordner |

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=custom,CDCOnBoot=cdc" \
  VoiceDot_Waveshare
```

Abhängigkeiten sind der **Arduino ESP32 Core 3.x** und **Adafruit NeoPixel**.
esp-sr ist fest im Core gelinkt, `mp3_decoder.cpp` / `.h` liegen im
Sketch-Ordner — mehr braucht es nicht.

Die Wake-Word-Modelle liegen **nicht** in diesem Repository und werden einmalig
per USB nach `0xC10000` geschrieben; wie man sie baut, steht in
[INSTALL.md](INSTALL.md).

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
| GET/POST | `/api/alarm` | Wecker lesen, stellen, löschen |
| GET/POST | `/api/timer` | Timer lesen, stellen, löschen |
| POST | `/api/alarm/briefing-test` | Briefing sofort sprechen |
| GET/POST | `/api/groups` | Gruppen lesen und speichern |
| GET | `/api/ha/entities?q=…` | Entitäten in Home Assistant suchen |
| POST | `/api/update/check` | Releases von GitHub holen |
| POST | `/api/update/install?tag=…` | Diese Version installieren |
| GET | `/api/hardware/mic-info` | Mikrofonpegel und Verstärkung |
| POST | `/api/hardware/led-test?phase=…` | Eine LED-Phase vorführen |
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

**„Spiele …" bleibt immer beim Radio.** Ist der Name nicht in der Liste, fragt
das Gerät zurück, statt die Bitte an Home Assistant durchzureichen — der
Assistent kann kein Radio abspielen und würde nur eine Entschuldigung
zurückgeben:

```text
„Spiele Radio Paloma"
   → „Den Sender kenne ich nicht. Ich habe Energy Wien, TechnoBase.FM
      oder ORF Hitradio Ö3. Welchen soll ich spielen?"
„Energy Wien"
   → spielt
```

Die Antwort darf der blanke Name sein, ohne „spiele" davor. „Abbrechen",
„nichts" oder „egal" beenden die Nachfrage; ein zweiter unbekannter Name auch,
damit es nicht im Kreis läuft. Gehört wird die Antwort 30 Sekunden lang, und
nur wenn **Rückfragen fortsetzen** eingeschaltet ist — sonst wird die Liste nur
vorgelesen.

Auch „Spiele" allein, ohne Namen, führt zur Nachfrage.
Gesprochener Name und gespeicherter Name werden vorher auf gemeinsamen Nenner
gebracht — klein geschrieben, Umlaute ausgeschrieben, Satzzeichen weg —, und
wenn nichts exakt passt, gewinnt der Sender mit den meisten übereinstimmenden
Wörtern. Ein einzelnes gemeinsames Wort in einer mehrwortigen Anfrage reicht
dafür nicht — das wäre geraten, und geraten wird lieber nachgefragt.

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

## Wecker und Morning Briefing

```text
„Alexa"  →  „Stelle den Wecker auf sieben Uhr dreißig"
„Alexa"  →  „Wecker löschen"
```

Gestellt wird **lokal**, ohne Home Assistant: die Zeit steht in der eigenen
Konfiguration, die RTC hält sie über einen Stromausfall, und der Wecker
klingelt auch dann, wenn das Netz gerade weg ist.

Die gesprochene Zeit wird Wort für Wort gelesen, nicht als Zeichenkette
durchsucht — „ein", „eine", „einen" und „eins" zählen alle als 1, und die
Minuten gehören nur dann zur Stunde, wenn sie unmittelbar folgen. Ohne das
verschluckt „sieben Uhr dreißig" zuverlässig seine Minuten.

**Das Briefing ist eine Anweisung, kein Text.** Was im Webinterface unter
*Wecker* steht, geht als Aufgabe an deine Assist-Pipeline — das Sprachmodell
holt sich Wetter, Kalender und Zustände selbst. Zum Beispiel:

```text
Wünsche freundlich einen guten Morgen und erwähne Auffälligkeiten der Nacht.
Stelle den Wetterbericht von heute vor und erläutere, was wir anziehen sollen.
Lies die ersten drei Termine von heute aus Alex' Kalender vor; falls es Termine
am Schulkalender gibt, erwähne diese explizit.
```

Davor läuft ein frei wählbarer Weckton aus den hochgeladenen Klängen. Ist
keiner gewählt, beginnt das Gerät direkt zu sprechen.

```text
GET  /api/alarm                      Zustand, Restzeit, Weckton, Briefing
POST /api/alarm?time=07:30&daily=1   stellen
POST /api/alarm?clear=1              löschen
POST /api/alarm/briefing-test        Briefing sofort sprechen
```

---

## Timer

```text
„Alexa"  →  „Stelle den Timer auf zehn Minuten"
„Alexa"  →  „Wie lange noch?"        →  „Noch acht Minuten und zwölf Sekunden."
„Alexa"  →  „Timer löschen"
```

Ebenfalls lokal, ebenfalls mit einem frei wählbaren Ton beim Ablauf — ohne
gewählten Ton wird das Ende angesagt. Der Zustand steht unter `/api/timer` und
damit auch in Home Assistant zur Verfügung.

---

## Gruppensteuerung

```text
„Alexa"  →  „Schalte im Obergeschoß das Licht aus"
„Alexa"  →  „Erdgeschoß Rollo öffnen"
```

Eine Gruppe ist ein Name und eine Liste von Entitäten, gepflegt im
Webinterface unter *Gruppen*. „Bearbeiten" öffnet ein Fenster: links die Suche
über alle Entitäten deiner Home-Assistant-Instanz, rechts die aufgenommenen
als Kacheln mit einem „x" zum Entfernen.

Erkannt wird über gemeinsame Wörter zwischen Gesagtem und Gruppennamen, und
zwar **wortweise**: ein `indexOf("aus")` würde in „Außenbeleuchtung" fündig und
das Licht ausschalten, statt es einzuschalten. Passt keine Gruppe, geht die
Bitte unverändert an Home Assistant weiter — die Gruppensteuerung nimmt dem
Assistenten nichts weg, sie kommt ihm nur zuvor, wo sie sicher ist.

```text
GET  /api/groups        Gruppen mit ihren Entitäten
POST /api/groups        Gruppen speichern (JSON)
GET  /api/ha/entities?q=licht    Entitäten suchen
```

---

## Die Oberfläche

Alles unter `http://<gerätename>.local`. Links ein Menübaum, oben rechts der
Umschalter zwischen hellem und dunklem Thema, unten eine Leiste mit
**Speichern**, die auf jeder Seite stehen bleibt.

Die Hauptseite zeigt Status, erkanntes Board, Hardware-Diagnose, den
Serial-Log der letzten 200 Zeilen und die Konfiguration. Alles Weitere liegt in
eigenen Seiten: Audio-Pipeline, Wake-Word, Tag und Nacht, Klänge, Webradio,
Wecker, Timer, Gruppen, Mehrere VoiceDots, Firmware.

**Speichern sichert immer die ganze Konfiguration**, nicht nur die sichtbare
Seite — die Einstellungen sind ein Formular über mehrere Seiten hinweg, und nur
das Sichtbare zu speichern würde den Rest stillschweigend zurücksetzen.

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
- Für die Verbindung zu GitHub muss die Wake-Word-Erkennung kurz weichen: TLS
  braucht rund 43 kB internen RAM, die Erkennung hält rund 73 kB davon, und
  beides zusammen passt nicht. Sie pausiert deshalb für die Suche (gemessen
  1,1 s) und für die Dauer eines Downloads und kommt danach von selbst zurück.
- Kein Barge-in und keine Erkennung, solange ein Update läuft.
- **Die API ist unauthentifiziert** — inklusive OTA. VoiceDot gehört in ein
  vertrauenswürdiges Netz.

---

## Lizenz

Diese Firmware steht unter der **GNU General Public License v3.0** — der volle
Text liegt in [LICENSE](LICENSE).

Die Wahl ist keine Geschmacksfrage: `mp3_decoder.cpp` und `mp3_decoder.h`
stammen aus dem Umfeld von
[ESP32-audioI2S](https://github.com/schreibfaul1/ESP32-audioI2S) (GPL-3.0) und
beruhen laut Kopfzeile auf dem **Helix MP3 Decoder** von RealNetworks
(RPSL/RCSL). Wer diese Dateien mitverteilt, verteilt fremden Code unter
fremden Bedingungen mit; GPL-3.0 für das Ganze ist der konfliktfreie Weg.

Nicht enthalten und nicht mitverteilt sind Espressifs Wake-Word-Modelle
(`srmodels.bin`) — die baust du dir aus dem esp-sr-Release selbst, siehe
[INSTALL.md](INSTALL.md).

---

## Home-Assistant-Integration

Für die komfortable Anbindung gibt es eine HACS-Integration:
[hass_voicedot-waveshare_hacs_plugin](https://github.com/xCite1986/hass_voicedot-waveshare_hacs_plugin)
— Erkennung per mDNS, ein Gerät je VoiceDot, Sensoren, Einstellungen und ein
Dienst `voicedot.announce` für Ansagen.
