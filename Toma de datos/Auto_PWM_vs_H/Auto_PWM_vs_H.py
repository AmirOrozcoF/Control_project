import serial
import serial.tools.list_ports
import time

# Detectar puerto Arduino automáticamente
def find_arduino():
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Arduino' in port.description or 'CH340' in port.description or 'USB' in port.description:
            return port.device
    return None

# Buscar puerto
puerto = find_arduino()

if puerto is None:
    print("No se encontró Arduino. Puertos disponibles:")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device}: {port.description}")
    puerto = input("Ingresa el puerto manualmente (ej: COM3): ")

print(f"Conectando a {puerto}...")

try:
    ser = serial.Serial(puerto, 9600, timeout=1)
    time.sleep(2)  # Esperar que Arduino reinicie después de la conexión
    
    print("Capturando datos... (presiona Ctrl+C para detener)")
    
    with open('3Escalones.csv', 'w') as f:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(line)
                    f.write(line + '\n')
                    f.flush()
                    
except KeyboardInterrupt:
    print("\nCaptura detenida por el usuario")
except Exception as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print("Puerto cerrado")