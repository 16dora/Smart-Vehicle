"""Run with ESP-IDF Python: python serial_test.py --port COM4."""
import argparse
import time
import serial


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='COM4')
    args = parser.parse_args()
    with serial.Serial(port=None, baudrate=115200, timeout=0.2) as port:
        port.dtr = False
        port.rts = False
        port.port = args.port
        port.open()
        # Allow any reset on open to finish, then discard startup output.
        time.sleep(2)
        port.reset_input_buffer()
        messages = ['ping', 'smart_vehicle_0123456789', 'x' * 127, 'x' * 128, 'after_overflow']
        expected = ['ECHO ' + item if len(item) <= 127 else
                    'ERROR line too long (max 127 bytes)' for item in messages]
        received = []
        heartbeats = []
        for message, reply in zip(messages, expected):
            port.write((message + '\n').encode('ascii'))
            port.flush()
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                line = port.readline().decode('ascii', errors='replace').strip()
                if line:
                    print(line)
                if line.startswith('HEARTBEAT '):
                    heartbeats.append(line)
                if line == reply:
                    received.append(line)
                    break
            else:
                raise RuntimeError('Missing reply: ' + reply)
        deadline = time.monotonic() + 4
        while len(heartbeats) < 3 and time.monotonic() < deadline:
            line = port.readline().decode('ascii', errors='replace').strip()
            if line.startswith('HEARTBEAT '):
                print(line)
                heartbeats.append(line)
        if len(heartbeats) < 3:
            raise RuntimeError('Expected at least three heartbeats')
        print(f'PASS: {len(received)} replies, {len(heartbeats)} heartbeats')


if __name__ == '__main__':
    main()
