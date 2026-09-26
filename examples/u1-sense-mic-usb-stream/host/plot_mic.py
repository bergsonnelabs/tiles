#!/usr/bin/env python3
"""
Live audio plotter for the Sense.MIC streaming demo.

Connects to the Core.ST.L4 USB serial, sends 's' to start streaming,
and plots the ADC waveform in real time using matplotlib.

Usage:
    python3 plot_mic.py [/dev/tty.usbmodemXXXX]

Press Ctrl+C to stop streaming and exit.
"""

import sys
import glob
import time
import struct
import signal
import threading
import collections

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# --- Configuration ---
BAUD = 115200
WINDOW_SAMPLES = 10000      # Rolling waveform window
SAMPLES_PER_PKT = 30        # Must match firmware
PKT_SIZE = 2 + SAMPLES_PER_PKT * 2  # 62 bytes
SYNC = bytes([0xAA, 0x55])
PLOT_INTERVAL_MS = 33        # ~30 fps


def find_port():
    """Auto-detect the USB serial port."""
    ports = glob.glob("/dev/tty.usbmodem*")
    if not ports:
        print("No USB serial device found. Is the Core.ST.L4 connected?")
        sys.exit(1)
    if len(ports) > 1:
        print(f"Multiple ports found: {ports}")
        print(f"Using {ports[0]}")
    return ports[0]


class MicStream:
    def __init__(self, port):
        self.ser = serial.Serial(port, BAUD, timeout=0.1)
        self.ser.dtr = True
        self.ser.rts = True

        self.samples = collections.deque(maxlen=WINDOW_SAMPLES)
        self.rate_text = "-- sps"
        self.dc_offset = 0
        self.running = True
        self.total_samples = 0
        self.total_packets = 0
        self.start_time = None

        # Pre-fill with zeros
        for _ in range(WINDOW_SAMPLES):
            self.samples.append(0)

    def start(self):
        """Send 's' to start streaming and begin reader thread."""
        time.sleep(0.5)
        # Drain any stale data
        self.ser.reset_input_buffer()
        self.ser.write(b"s")
        self.start_time = time.time()
        self.reader_thread = threading.Thread(target=self._reader, daemon=True)
        self.reader_thread.start()

    def stop(self):
        """Send 'x' to stop streaming."""
        self.running = False
        try:
            self.ser.write(b"x")
            time.sleep(0.1)
            self.ser.close()
        except Exception:
            pass

    def _reader(self):
        """Background thread: read and parse binary packets from serial."""
        buf = bytearray()

        while self.running:
            try:
                chunk = self.ser.read(max(1, self.ser.in_waiting or 1))
            except Exception:
                break
            if not chunk:
                continue

            buf.extend(chunk)

            # Process all complete packets in the buffer
            while len(buf) >= 2:
                # Check for text line (rate messages start with '#')
                if buf[0] == ord("#"):
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break  # Incomplete line, wait for more data
                    line = buf[:nl].decode("utf-8", errors="replace").strip()
                    buf = buf[nl + 1 :]
                    self._handle_text(line)
                    continue

                # Look for sync header
                idx = buf.find(SYNC)
                if idx < 0:
                    # No sync found — discard all but last byte (might be partial sync)
                    buf = buf[-1:]
                    continue
                if idx > 0:
                    # Discard bytes before sync
                    buf = buf[idx:]

                # Check if full packet available
                if len(buf) < PKT_SIZE:
                    break

                # Extract packet
                pkt = buf[:PKT_SIZE]
                buf = buf[PKT_SIZE:]

                # Parse samples (little-endian uint16)
                for i in range(SAMPLES_PER_PKT):
                    lo = pkt[2 + i * 2]
                    hi = pkt[2 + i * 2 + 1]
                    sample = lo | (hi << 8)
                    self.samples.append(sample)

                self.total_samples += SAMPLES_PER_PKT
                self.total_packets += 1

    def _handle_text(self, line):
        """Parse text messages from firmware."""
        if line.startswith("# rate="):
            self.rate_text = line[2:]  # "rate=XXXX sps"
        elif line.startswith("# dc_offset="):
            try:
                self.dc_offset = int(line.split("=")[1])
            except ValueError:
                pass
        # Print all text messages to console
        print(line)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    print(f"Connecting to {port}...")

    stream = MicStream(port)

    # Handle Ctrl+C gracefully
    def on_sigint(sig, frame):
        print("\nStopping...")
        stream.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sigint)

    print("Starting stream...")
    stream.start()

    # Set up matplotlib
    fig, ax = plt.subplots(figsize=(14, 4))
    fig.canvas.manager.set_window_title("Sense.MIC Live")
    (line,) = ax.plot([], [], linewidth=0.5, color="#2196F3")
    ax.set_xlim(0, WINDOW_SAMPLES)
    ax.set_ylim(0, 4095)
    ax.set_xlabel("Sample")
    ax.set_ylabel("ADC (12-bit)")
    ax.grid(True, alpha=0.3)
    title = ax.set_title("Sense.MIC — waiting for data...", fontsize=11)

    def update(frame):
        data = list(stream.samples)
        line.set_data(range(len(data)), data)

        # Fixed Y axis — practical range for mic signal
        ax.set_ylim(0, 1500)

        elapsed = time.time() - stream.start_time if stream.start_time else 0
        measured = (
            f"{stream.total_samples / elapsed:.0f}"
            if elapsed > 0.5
            else "--"
        )
        title.set_text(
            f"Sense.MIC — {stream.rate_text}  |  "
            f"measured: {measured} sps  |  "
            f"pkts: {stream.total_packets}"
        )
        return (line, title)

    ani = animation.FuncAnimation(
        fig, update, interval=PLOT_INTERVAL_MS, blit=False, cache_frame_data=False
    )

    def on_close(event):
        stream.stop()

    fig.canvas.mpl_connect("close_event", on_close)

    plt.tight_layout()
    plt.show()

    stream.stop()


if __name__ == "__main__":
    main()
