from pymodbus.client.serial import ModbusSerialClient

# Verbinde mit dem Modbus RTU Greifer (angepasster Port!)
client = ModbusSerialClient(
    method='rtu',
    port='/dev/ttyUSB0',  # Falls dein Adapter z. B. /dev/ttyUSB1 ist, ändere es hier
    baudrate=115200,
    timeout=1,
    parity='N',
    stopbits=1,
    bytesize=8
)

if client.connect():
    print("Verbindung erfolgreich!")

    # Greifer aktivieren
    client.write_register(0x03E8, 0x0100, unit=9)  # rACT=1 (Gripper Activation)
    
    # Greifer schließen
    client.write_register(0x03E8, 0x0900 | 255, unit=9)  # rGTO=1, rPR=255 (Ganz schließen)
    
    # Wartezeit zum Greifen
    import time
    time.sleep(2)

    # Greifer öffnen
    client.write_register(0x03E8, 0x0900 | 0, unit=9)  # rGTO=1, rPR=0 (Ganz öffnen)

    client.close()
else:
    print("Fehler: Verbindung fehlgeschlagen!")
