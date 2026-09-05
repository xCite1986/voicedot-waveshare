# Installation

Von der leeren Platine bis zum sprechenden VoiceDot. Einmal USB, danach nie
wieder — Firmware-Updates laufen über das Netz.

Wer das Gerät nur aktualisieren will, springt direkt zu
[Updates danach](#updates-danach).

---

## 1. Was du brauchst

| | |
|---|---|
| Board | Waveshare ESP32-S3-AUDIO-Board (ESP32-S3R8, 16 MB Flash, 8 MB PSRAM) |
| Kabel | USB-C, **Datenkabel** — ein reines Ladekabel meldet kein Gerät an |
| Software | [Arduino IDE 2.x](https://www.arduino.cc/en/software) oder `arduino-cli` |
| Netz | 2,4-GHz-WLAN; das Board kann kein 5 GHz |
| Dienst | Home Assistant mit eingerichteter Assist-Pipeline |

Die Assist-Pipeline in Home Assistant sollte vorher **im Browser
funktionieren** (Einstellungen → Sprachassistenten → Ausprobieren). Alles, was
dort nicht geht, geht auch auf dem Gerät nicht — dann liegt es nicht am
VoiceDot.

---

## 2. Arduino IDE vorbereiten

1. **ESP32-Core 3.x** installieren: Datei → Einstellungen → *Zusätzliche
   Boardverwalter-URLs*:

   ```text
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```

   Danach Werkzeuge → Board → Boardverwalter → **esp32 by Espressif**
   installieren. Entwickelt und getestet mit **3.3.8**.

2. **Adafruit NeoPixel** über die Bibliotheksverwaltung installieren.

3. **esp-sr** braucht keine Installation — die Erkennung ist fest im Core
   gelinkt. Die Wrapper-Bibliothek `ESP_SR` wird bewusst *nicht* verwendet,
   diese Firmware spricht direkt mit der AFE.

Mehr ist es nicht. `mp3_decoder.cpp` und `mp3_decoder.h` liegen im
Sketch-Ordner und werden mitkompiliert.

---

## 3. Board-Einstellungen

Der Ordner `VoiceDot_Waveshare/` muss **genau so heißen wie die `.ino`** —
sonst findet die IDE den Sketch nicht.

| Einstellung | Wert |
|---|---|
| Board | ESP32S3 Dev Module |
| Flash Size | 16MB (128Mb) |
| PSRAM | OPI PSRAM |
| USB CDC On Boot | Enabled |
| **Partition Scheme** | **Custom** |

*Custom* nimmt die `partitions.csv` aus dem Sketch-Ordner. Sie enthält neben
den beiden 3-MB-OTA-Slots eine eigene `model`-Partition ab `0xC10000` für die
Wake-Word-Modelle.

Bewusst **nicht** das fertige Schema „ESP SR 16M": das schreibt bei jedem
Upload Espressifs Standard-Blob dorthin und überbügelt die eigenen Modelle.

Auf der Kommandozeile:

```bash
arduino-cli compile \
  --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=custom,CDCOnBoot=cdc" \
  VoiceDot_Waveshare
```

---

## 4. Wake-Word-Modelle erzeugen

Die Modelle liegen **nicht** in diesem Repository — Espressifs Binärdaten
werden hier nicht weiterverteilt. Sie lassen sich aber reproduzierbar bauen,
ohne kompletten ESP-IDF-Build.

1. Arduino-Core 3.3.8 entspricht **esp-sr 2.4.1**; der Modellsatz ist identisch
   (61 Einträge — 2.4.2 hat drei mehr, 2.3.1 weicht ab). Passend zur eigenen
   Core-Version wählen.

2. Aus dem Release-ZIP von [esp-sr](https://github.com/espressif/esp-sr) nur
   diese vier Ordner in einen leeren Ordner `models_src/` kopieren:

   ```text
   model/wakenet_model/wn9_alexa
   model/wakenet_model/wn9_jarvis_tts
   model/wakenet_model/wn9_computer_tts
   model/vadnet_model/vadnet1_medium
   ```

3. Mit dem `pack_model.py` aus demselben Release packen:

   ```bash
   python model/pack_model.py -m models_src -o srmodels.bin
   ```

**`vadnet1_medium` muss mit ins Blob.** Die AFE ruft
`esp_vadn_handle_from_name` auf und der Core ist mit
`CONFIG_SR_VADN_VADNET1_MEDIUM=y` gebaut — fehlt das Modell, scheitert die
AFE-Erzeugung und das Gerät startet ohne Stichworterkennung.

MultiNet (`mn7_en`, 2,7 MB) fehlt dagegen absichtlich: diese Firmware nutzt
keine Kommandoerkennung, das spart Platz und PSRAM.

---

## 5. Flashen

1. **Firmware** aus der Arduino IDE hochladen (Pfeil-Symbol), oder:

   ```bash
   arduino-cli upload -p COM4 \
     --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=16M,PartitionScheme=custom,CDCOnBoot=cdc" \
     VoiceDot_Waveshare
   ```

2. **Modelle** einmalig an die feste Adresse schreiben:

   ```bash
   esptool --chip esp32s3 --port COM4 write-flash 0xC10000 srmodels.bin
   ```

   Das überlebt danach jeden Firmware-Upload und jedes Netz-Update.

Meldet sich kein Port, das Board mit gedrückter **BOOT**-Taste anstecken und
die Taste erst nach dem Einstecken loslassen.

---

## 6. Erste Inbetriebnahme

1. Starten. Ohne gespeichertes WLAN öffnet der VoiceDot einen
   Setup-Accesspoint **`VoiceDot-XXXXXX`**, Passwort **`voicedot`**. Die
   Oberfläche liegt unter `http://192.168.4.1` und kommt auf den meisten
   Geräten von selbst als Captive Portal hoch. Der LED-Ring leuchtet dabei
   orange.
2. WLAN auswählen, Passwort eintragen, speichern — das Board startet neu.
3. Danach erreichbar unter `http://<gerätename>.local`, voreingestellt
   `http://voicedot.local`.
4. Unter **Home Assistant** die URL und einen **Long-Lived Access Token**
   eintragen (in HA: Profil → Sicherheit → ganz unten) und *Verbindung testen*.
5. *Pipelines aus HA laden* und die gewünschte Assist-Pipeline wählen.
6. Einmal eine Frage stellen — dabei lernt der VoiceDot Engine, Sprache und
   Stimme der Pipeline.
7. **Ansagen erzeugen** drücken. Damit werden die Bestätigungen („Was kann ich
   für dich tun?") einmalig über deine HA-TTS erzeugt und lokal gespeichert;
   danach kommen sie ohne Netz und ohne Wartezeit.

Läuft. Ab hier braucht es kein Kabel mehr.

---

## 7. Home Assistant anbinden

Für die bequeme Anbindung gibt es eine HACS-Integration:
**[hass_voicedot-waveshare_hacs_plugin](https://github.com/xCite1986/hass_voicedot-waveshare_hacs_plugin)**

In HACS als benutzerdefiniertes Repository hinzufügen, installieren, HA neu
starten. Die Geräte werden per mDNS gefunden; je VoiceDot entsteht ein Gerät
mit Sensoren, Einstellungen, Wecker- und Timer-Zuständen und dem Dienst
`voicedot.announce`.

Ohne die Integration geht es auch — über `rest_command`:

```yaml
rest_command:
  voicedot_ansage:
    url: "http://voicedot.local/api/announce"
    method: POST
    content_type: "application/json; charset=utf-8"
    payload: '{"text": "{{ text }}"}'
```

---

## 8. Updates danach

Der VoiceDot holt sich neue Firmware selbst aus den
[Releases](../../releases) dieses Repositories: **Firmware → Nach Updates
suchen → installieren**. Geschrieben wird immer in die gerade nicht laufende
Partition, ein misslungenes Update kann das Gerät also nicht unbrauchbar
machen.

Während Suche und Download hört das Gerät **kurz nicht zu**: die
Verschlüsselung zu GitHub braucht denselben internen Speicher wie die
Wake-Word-Erkennung. Die Erkennung pausiert dafür rund eine Sekunde und kommt
von selbst zurück.

Einstellungen, Klänge, Sender und Gruppen überleben das Update. Die
Wake-Word-Modelle liegen in ihrer eigenen Partition und werden nicht angefasst
— nur wenn eine künftige Version *andere* Modelle braucht, muss `srmodels.bin`
einmal per USB nachgezogen werden.

Alternativ nimmt die Oberfläche unter **Firmware** auch eine `.bin` direkt
entgegen (Arduino IDE → Sketch → *Kompilierte Binärdatei exportieren*).

---

## Wenn etwas nicht geht

| Symptom | Ursache |
|---|---|
| Kein serieller Port | Ladekabel statt Datenkabel, oder BOOT-Taste beim Anstecken nötig |
| Startet, aber kein Stichwort | `srmodels.bin` fehlt oder liegt an der falschen Adresse — die Oberfläche sagt das unter *Hardware* |
| Findet das WLAN nicht | 5-GHz-Netz; das Board kann nur 2,4 GHz |
| `.local`-Adresse geht nicht | mDNS im Netz blockiert — die IP steht in der Fritzbox oder im Serial-Log |
| Assist antwortet mit Fehler | Erst die Pipeline in HA im Browser testen |
| Zwei Geräte antworten gleichzeitig | Subnetz-Broadcast blockiert (VLAN, Client-Isolation) — die Aushandlung braucht ihn |

Der Serial-Log steht auch ohne Kabel in der Oberfläche auf der Hauptseite und
unter `/api/log`.
