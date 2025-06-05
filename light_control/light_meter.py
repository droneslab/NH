import serial
import re
import time

def connect_serial(serial_port, baud_rate=9600):
    """
    Establishes a serial connection to the microcontroller.

    :param serial_port: The serial port (e.g., 'COM3' or '/dev/ttyUSB0').
    :param baud_rate: The baud rate of the serial connection (default: 9600).
    :return: Serial object if successful, None otherwise.
    """
    try:
        ser = serial.Serial(serial_port, baud_rate, timeout=1)
        print(f"Connected to {serial_port} at {baud_rate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Error: {e}")
        return None

def read_sensor_data(ser, sensor_model="TSL2591"):
    """
    Reads and parses sensor data from the serial connection.

    :param ser: Serial object.
    :param sensor_model: The sensor model (default: 'TSL2591').
    :return: Dictionary with parsed sensor values or None if invalid data.
    """
    if ser.in_waiting > 0:
        raw_data = ser.readline().decode('utf-8').strip()

        # Match the sensor data format using regex
        match = re.search(r"\[\s*(\d+) ms\s*\] IR:\s*(\d+)\s*Full:\s*(\d+)\s*Visible:\s*(\d+)\s*Lux:\s*([\d\.]+)", raw_data)
        
        if match:
            return {
                "timestamp_ms": float(match.group(1)),
                "IR": float(match.group(2)),
                "Full": float(match.group(3)),
                "Visible": float(match.group(4)),
                "Lux": float(match.group(5)),
                "Sensor": sensor_model
            }
    
    return None  # Return None if no valid data

def main():
    serial_port = "/dev/ttyACM1"  # Change this based on your system (e.g., "/dev/ttyUSB0" for Linux/macOS)
    baud_rate = 9600

    ser = connect_serial(serial_port, baud_rate)
    if not ser:
        return

    try:
        while True:
            sensor_data = read_sensor_data(ser)
            if sensor_data:
                print(sensor_data)
            time.sleep(0.5)  # Add a small delay to avoid overloading the CPU
    except KeyboardInterrupt:
        print("\nClosing connection.")
        ser.close()

if __name__ == "__main__":
    main()
