import serial
import serial.tools.list_ports
import re
import subprocess
import sys

# Regex pattern for expected data
DATA_PATTERN = re.compile(r"X:(\d+),Y:(\d+)")

# --- Helper Functions --- #

def list_com_ports():
    """Return all available COM ports on the system."""
    return [port.device for port in serial.tools.list_ports.comports()]

def test_com_port(port, baudrate=38400, timeout=1, max_lines=5):
    """
    Try reading a few lines from the COM port to see if it matches X:...,Y:...
    Returns True if match found, False otherwise.
    """
    try:
        with serial.Serial(port, baudrate=baudrate, timeout=timeout) as ser:
            for _ in range(max_lines):
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if DATA_PATTERN.match(line):
                    return True
        return False
    except serial.SerialException:
        return False

def connect_serial(port, baudrate=38400, timeout=1):
    """Connect to the serial COM port."""
    try:
        ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud.")
        return ser
    except serial.SerialException as e:
        print(f"Failed to connect: {e}")
        return None

def read_bt_data(ser):
    """Continuously read and print X,Y data from the COM port."""
    print("Start reading data. Press Ctrl+C to stop.")
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line:
                match = DATA_PATTERN.match(line)
                if match:
                    x, y = match.groups()
                    print(f"X = {x}, Y = {y}")
    except KeyboardInterrupt:
        print("\nStopped reading.")
    finally:
        ser.close()

def list_nearby_bt_devices():
    """List nearby Bluetooth devices using Windows PowerShell."""
    try:
        cmd = ["powershell", "-Command",
               "Get-PnpDevice -Class Bluetooth | Where-Object {$_.Status -eq 'OK'} | Select-Object -Property FriendlyName,DeviceID"]
        output = subprocess.check_output(cmd, text=True)
        devices = []
        for line in output.splitlines():
            line = line.strip()
            if line and "DeviceID" not in line and "FriendlyName" not in line:
                parts = line.split()
                if len(parts) >= 2:
                    name = " ".join(parts[:-1])
                    device_id = parts[-1]
                    devices.append((name, device_id))
        return devices
    except Exception as e:
        print("Failed to list Bluetooth devices:", e)
        return []

def pair_with_btcli(mac, pin="1234"):
    """Pair a Bluetooth device using btpair CLI tool."""
    try:
        cmd = ["btpair", "-b", mac, pin]
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, text=True, timeout=15)
        if "success" in out.lower() or "paired" in out.lower():
            print("Paired successfully!")
            return True
        else:
            print("Pairing output:", out)
            return False
    except FileNotFoundError:
        print("btpair not found. Install Bluetooth Command Line Tools.")
        return False
    except subprocess.CalledProcessError as e:
        print("Pairing failed:", e.output)
        return False

# --- Main Workflow --- #
if __name__ == "__main__":
    print("Scanning all COM ports for Bluetooth data...")
    hc05_port = None

    for port in list_com_ports():
        print(f"Testing {port}...")
        if test_com_port(port):
            hc05_port = port
            print(f"Found matching device on {hc05_port}")
            break

    # If no COM port matches, let user pair a device
    if hc05_port is None:
        print("No Bluetooth device sending expected data found.")
        nearby_devices = list_nearby_bt_devices()
        if not nearby_devices:
            print("No nearby Bluetooth devices detected. Exiting.")
            sys.exit(0)

        print("Nearby Bluetooth devices:")
        for i, (name, _) in enumerate(nearby_devices):
            print(f"{i}: {name}")

        choice = int(input("Select a device to pair: "))
        device_name, device_mac = nearby_devices[choice]
        pin = input("Enter Bluetooth PIN (default 1234): ").strip() or "1234"
        paired = pair_with_btcli(device_mac, pin)

        if not paired:
            print("Failed to pair. Exiting.")
            sys.exit(0)

        print("Rescanning COM ports after pairing...")
        for port in list_com_ports():
            print(f"Testing {port}...")
            if test_com_port(port):
                hc05_port = port
                print(f"Found matching device on {hc05_port}")
                break

    if hc05_port:
        serial_conn = connect_serial(hc05_port)
        if serial_conn:
            read_bt_data(serial_conn)
    else:
        print("No device found sending expected data. Exiting.")
