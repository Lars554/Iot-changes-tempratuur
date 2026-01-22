import serial
import json
import os
import time
from datetime import datetime

SERIAL_PORT = "/dev/ttyACM0"
BAUDRATE = 9600
DB_FILE = "adc_database.json"

# Seriële poort openen
try:
    ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    time.sleep(2)  # Arduino reset-tijd
except serial.SerialException as e:
    print("Kan seriële poort niet openen:", e)
    exit(1)

# Database initialiseren
if not os.path.exists(DB_FILE):
    with open(DB_FILE, "w") as f:
        json.dump([], f)

print("UART lezen gestart...")

while True:
    try:
        line = ser.readline().decode("utf-8", errors="ignore").strip()

        if not line:
            continue  # lege regels negeren

        # Probeer 1 getal te lezen
        value = int(line)

        entry = {
            "Tijd": datetime.now().isoformat(),
            "Graden C°": value
        }

        with open(DB_FILE, "r") as f:
            data = json.load(f)

        data.append(entry)

        with open(DB_FILE, "w") as f:
            json.dump(data, f, indent=2)

        print("Opgeslagen:", entry)

    except ValueError:
        # Geen geldig getal ontvangen
        print("Ongeldige data:", line)

    except serial.SerialException as e:
        print("Seriële verbinding verbroken:", e)
        break

    except Exception as e:
        print("Onverwachte fout:", e)
