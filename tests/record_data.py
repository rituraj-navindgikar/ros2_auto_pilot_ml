import serial
import csv

# Configure the serial connection
arduino_port = "/dev/ttyACM0"  # Replace with your Arduino's port (e.g., /dev/ttyUSB0 on Linux/Mac)
baud_rate = 115200     # Match the baud rate in your Arduino code
output_file = "lidar_data_parsed.csv"  # File to save the parsed data

try:
    # Open the serial port
    ser = serial.Serial(arduino_port, baud_rate, timeout=1)
    print(f"Connected to {arduino_port} at {baud_rate} baud")

    # Open the CSV file for logging
    with open(output_file, mode="w", newline="") as file:
        writer = csv.writer(file)
        # Write the header with only the essential fields
        writer.writerow(["Timestamp (ms)", "RPM", "Angle (degrees)", "Distance (mm)", "Intensity"])

        print("Logging lidar data...")
        while True:
            try:
                # Read human-readable data from the serial port
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(line)  # Print the data for debugging
                    # Parse the simplified format
                    try:
                        parts = line.split(", ")
                        # Extract individual fields
                        timestamp = int(parts[0].split(": ")[1])
                        rpm = int(parts[1].split(": ")[1])
                        angle = float(parts[2].split(": ")[1].replace("°", ""))
                        distance = int(parts[3].split(": ")[1].replace(" mm", ""))
                        intensity = int(parts[4].split(": ")[1])

                        # Save the parsed data to the CSV
                        writer.writerow([timestamp, rpm, angle, distance, intensity])
                    except Exception as e:
                        print(f"Error parsing line: {line} - {e}")
            except KeyboardInterrupt:
                print("Logging stopped by user.")
                break

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
    print("Serial connection closed.")

