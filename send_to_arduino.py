import time
import struct
import numpy as np
import serial
from serial.tools import list_ports
from datetime import datetime
from pathlib import Path

Synk_1 = 0xAA
Synk_2 = 0x55
Kalibrerings_kommando = b"C"
Antal_motorer = 6

# 2 sync + 2 sequence + 2 dt + 12 position bytes + 12 velocity bytes + 1 checksum
paket_storlek = 31


def resolve_serial_port(requested_port: str | None) -> str:
    """Return a usable serial port or raise a clear error."""
    if requested_port:
        return requested_port

    ports = [port.device for port in list_ports.comports()]

    if len(ports) == 1:
        print(f"Auto-selected serial port: {ports[0]}")
        return ports[0]

    if not ports:
        raise RuntimeError(
            "No serial ports were found. Connect the Arduino and set config.SERIAL_PORT if needed."
        )

    raise RuntimeError(
        "No SERIAL_PORT was configured and multiple serial ports are available: "
        + ", ".join(ports)
        + ". Set config.SERIAL_PORT to the Arduino port."
    )


def berakna_checksum(data: bytes) -> int:
    """Simple checksum: sum all bytes and keep the lower 8 bits."""
    return sum(data) & 0xFF


def bygg_paket(sekvens: int, dt_ms: int, positioner_mm, hastigheter_mm_s) -> bytes:
    """
    Build one packet for Arduino.

    Packet format:
      uint8  sync1
      uint8  sync2
      uint16 sequence
      uint16 dt_ms
      int16  positions[6]
      int16  velocities[6]
      uint8  checksum
    """

    if len(positioner_mm) != Antal_motorer:
        raise ValueError("positioner_mm must have length 6")

    if len(hastigheter_mm_s) != Antal_motorer:
        raise ValueError("hastigheter_mm_s must have length 6")

    pos = [int(np.clip(x, -32768, 32767)) for x in positioner_mm]
    vel = [int(np.clip(x, -32768, 32767)) for x in hastigheter_mm_s]

    payload = struct.pack(
        "<BBHH6h6h",
        Synk_1,
        Synk_2,
        sekvens & 0xFFFF,
        dt_ms & 0xFFFF,
        *pos,
        *vel,
    )

    checksum = berakna_checksum(payload)
    paket = payload + struct.pack("<B", checksum)

    if len(paket) != paket_storlek:
        raise RuntimeError(
            f"Packet size mismatch: got {len(paket)}, expected {paket_storlek}"
        )

    return paket


class SerialMonitor:
    """Monitor and log serial messages from Arduino."""

    def __init__(self, log_file: str | None = None):
        self.log_file = log_file
        self.buffer = ""
        self.message_count = 0
        self.received_messages = []

        if self.log_file:
            self.log_file_path = Path(self.log_file)
            self.log_file_path.parent.mkdir(parents=True, exist_ok=True)
            self._write_log_header()

    def _write_log_header(self):
        with open(self.log_file_path, "w", encoding="utf-8") as f:
            f.write("=== Arduino Serial Monitor Log ===\n")
            f.write(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'=' * 50}\n\n")

    def read_and_process(self, ser: serial.Serial) -> list[str]:
        """Read all available serial data and return complete lines."""
        messages = []

        while True:
            try:
                if ser.in_waiting > 0:
                    byte = ser.read(1)
                    if byte:
                        self.buffer += byte.decode("utf-8", errors="replace")
                else:
                    break
            except Exception as e:
                print(f"Error reading serial: {e}")
                break

        lines = self.buffer.split("\n")

        # Keep incomplete final line
        self.buffer = lines[-1] if lines[-1] else ""

        for line in lines[:-1]:
            line = line.strip()
            if line:
                self.message_count += 1
                messages.append(line)
                self.received_messages.append((datetime.now(), line))
                self._log_message(line)

        return messages

    def _log_message(self, message: str):
        if self.log_file:
            with open(self.log_file_path, "a", encoding="utf-8") as f:
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                f.write(f"[{timestamp}] {message}\n")

    def display_messages(self, messages: list[str]):
        if not messages:
            return

        colors = {
            "CALIBRATION": "\033[92m",
            "ERROR": "\033[91m",
            "WARNING": "\033[93m",
            "DEBUG": "\033[94m",
            "Current": "\033[96m",
            "RESET": "\033[0m",
        }

        for msg in messages:
            color = colors["RESET"]

            if "CALIBRATION" in msg:
                color = colors["CALIBRATION"]
            elif "ERROR" in msg or "Bad" in msg:
                color = colors["ERROR"]
            elif "WARNING" in msg:
                color = colors["WARNING"]
            elif "Current" in msg:
                color = colors["Current"]
            elif "DEBUG" in msg:
                color = colors["DEBUG"]

            timestamp = datetime.now().strftime("%H:%M:%S")
            print(f"{color}[{timestamp}] RX: {msg}{colors['RESET']}")

    def get_summary(self) -> str:
        summary = f"\n{'=' * 60}\n"
        summary += "Serial Monitor Summary\n"
        summary += f"Total messages received: {self.message_count}\n"

        calibration_msgs = sum(
            1 for _, msg in self.received_messages if "CALIBRATION" in msg
        )
        error_msgs = sum(
            1
            for _, msg in self.received_messages
            if "ERROR" in msg or "Bad" in msg
        )
        debug_msgs = sum(
            1 for _, msg in self.received_messages if "Current" in msg
        )

        if calibration_msgs > 0:
            summary += f"Calibration messages: {calibration_msgs}\n"

        if error_msgs > 0:
            summary += f"Error messages: {error_msgs}\n"

        if debug_msgs > 0:
            summary += f"Debug updates: {debug_msgs}\n"

        summary += f"{'=' * 60}\n"
        return summary


def _compute_sample_indices(num_samples: int, step: int) -> list[int]:
    """Return the IK sample indices that should be sent."""
    if num_samples <= 0:
        return []

    indices = list(range(0, num_samples, step))

    if indices[-1] != num_samples - 1:
        indices.append(num_samples - 1)

    return indices


def _estimate_velocities_from_positions(positioner_mm: np.ndarray, sample_rate: float) -> np.ndarray:
    """
    Fallback velocity estimate if ik_result has no leg_velocities.

    Uses np.gradient so velocity array has the same length as position array.
    """
    return np.gradient(positioner_mm, axis=0) * sample_rate

def stream_unit_step_to_arduino(
    port: str | None,
    baudrate: int = 115200,
    start_position_mm: float = 50.0,
    end_position_mm: float = 60.0,
    step_motor: int | None = None,
    hold_start_s: float = 3.0,
    hold_end_s: float = 8.0,
    packet_dt_ms: int = 50,
    startup_delay_s: float = 3.0,
    calibrate_before_stream: bool = True,
    calibration_wait_s: float = 18.0,
    log_serial: str | None = None,
):
    """
    Send a unit-step-like actuator reference to Arduino.

    start_position_mm:
        Initial actuator position for all motors.

    end_position_mm:
        Step target position.

    step_motor:
        None  -> step all motors.
        0..5  -> step only one motor, where 0 = motor A, 5 = motor F.

    Velocity reference is zero during the whole test.
    """

    if step_motor is not None and not (0 <= step_motor < Antal_motorer):
        raise ValueError("step_motor must be None or an integer from 0 to 5")

    start_position_mm = float(np.clip(start_position_mm, 0.0, 100.0))
    end_position_mm = float(np.clip(end_position_mm, 0.0, 100.0))

    port = resolve_serial_port(port)
    monitor = SerialMonitor(log_file=log_serial)

    start_positions = np.full(Antal_motorer, start_position_mm, dtype=np.int16)
    end_positions = np.full(Antal_motorer, start_position_mm, dtype=np.int16)

    if step_motor is None:
        end_positions[:] = int(round(end_position_mm))
    else:
        end_positions[step_motor] = int(round(end_position_mm))

    velocities = np.zeros(Antal_motorer, dtype=np.int16)

    n_start_packets = int(round(hold_start_s * 1000.0 / packet_dt_ms))
    n_end_packets = int(round(hold_end_s * 1000.0 / packet_dt_ms))

    print("Streaming unit step to Arduino")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Start position: {start_positions.tolist()} mm")
    print(f"End position:   {end_positions.tolist()} mm")
    print(f"Velocity ref:   {velocities.tolist()} mm/s")
    print(f"packet_dt_ms:   {packet_dt_ms}")
    print(f"hold_start_s:   {hold_start_s}")
    print(f"hold_end_s:     {hold_end_s}")

    with serial.Serial(port, baudrate, timeout=0.1, write_timeout=1.0) as ser:
        print(f"Waiting {startup_delay_s:.1f} seconds for Arduino startup...")
        time.sleep(startup_delay_s)

        startup_messages = monitor.read_and_process(ser)
        if startup_messages:
            print("\n--- Startup Messages ---")
            monitor.display_messages(startup_messages)
            print()

        if calibrate_before_stream:
            print("Requesting Arduino calibration...")
            ser.write(Kalibrerings_kommando)
            ser.flush()

            print(f"Waiting {calibration_wait_s:.1f} seconds for calibration...")
            time.sleep(calibration_wait_s)

            calibration_messages = monitor.read_and_process(ser)
            if calibration_messages:
                print("\n--- Calibration Messages ---")
                monitor.display_messages(calibration_messages)
                print()

            ser.reset_input_buffer()
            ser.reset_output_buffer()
            print("Calibration wait complete. Starting unit step.\n")

        sekvens = 0

        def send_repeated(position_array, count, label):
            nonlocal sekvens

            print(f"\n--- Sending {label}: {count} packets ---")

            next_send_time = time.perf_counter()

            for k in range(count):
                now = time.perf_counter()
                sleep_time = next_send_time - now

                if sleep_time > 0:
                    time.sleep(sleep_time)

                pkt = bygg_paket(
                    sekvens=sekvens,
                    dt_ms=packet_dt_ms,
                    positioner_mm=position_array,
                    hastigheter_mm_s=velocities,
                )

                ser.write(pkt)
                ser.flush()

                if k % max(1, count // 10) == 0 or k == count - 1:
                    print(
                        f"{label} seq={sekvens:5d} "
                        f"packet={k + 1:4d}/{count:4d} "
                        f"pos={position_array.tolist()}"
                    )

                incoming_messages = monitor.read_and_process(ser)
                if incoming_messages:
                    monitor.display_messages(incoming_messages)

                sekvens = (sekvens + 1) & 0xFFFF
                next_send_time += packet_dt_ms / 1000.0

        send_repeated(start_positions, n_start_packets, "START HOLD")
        send_repeated(end_positions, n_end_packets, "STEP HOLD")

        print("\nUnit step complete. Reading final messages...")
        time.sleep(0.5)

        final_messages = monitor.read_and_process(ser)
        if final_messages:
            print("--- Final Messages ---")
            monitor.display_messages(final_messages)

        print(monitor.get_summary())


def stream_ik_to_arduino(
    port: str | None,
    baudrate: int,
    ik_result,
    neutral_leg_length_m: float,
    actuator_center_mm: float = 50.0,
    actuator_min_mm: float = 0.0,
    actuator_max_mm: float = 100.0,
    send_every_nth: int = 5,
    startup_delay_s: float = 3.0,
    batch_size: int = 1,
    wait_between_batches_s: float = 0.0,
    calibrate_before_stream: bool = True,
    calibration_wait_s: float = 18.0,
    log_serial: str | None = None,
):
    """
    Stream IK result to Arduino as timed actuator packets.

    Each packet contains:
      - sequence number
      - dt_ms
      - 6 actuator positions in mm
      - 6 actuator velocities in mm/s

    Mapping:
      actuator_position_mm =
          (leg_length_mm - neutral_leg_length_mm) + actuator_center_mm

    The simplified Arduino controller uses the latest packet directly:
      - latest packet position = reference position
      - latest packet velocity = velocity feed-forward

    Negative velocities are allowed.
    """

    if batch_size < 1:
        raise ValueError("batch_size must be >= 1")

    if send_every_nth < 1:
        raise ValueError("send_every_nth must be >= 1")

    if len(ik_result.leg_lengths) == 0:
        raise ValueError("ik_result contains no samples")

    leg_mm = ik_result.leg_lengths * 1000.0
    neutral_mm = neutral_leg_length_m * 1000.0

    # Convert absolute leg lengths to actuator stroke positions.
    positioner_mm_float = (leg_mm - neutral_mm) + actuator_center_mm

    # Clip positions to actuator physical range.
    positioner_mm = np.round(positioner_mm_float)
    positioner_mm = np.clip(
        positioner_mm,
        actuator_min_mm,
        actuator_max_mm,
    ).astype(np.int16)

    # Velocities:
    # Prefer ik_result.leg_velocities if it exists.
    # Otherwise estimate from the actuator positions.
    if hasattr(ik_result, "leg_velocities") and ik_result.leg_velocities is not None:
        hastigheter_mm_s_float = ik_result.leg_velocities * 1000.0 *0.1
    else:
        print("WARNING: ik_result has no leg_velocities. Estimating velocities from positions.")
        hastigheter_mm_s_float = _estimate_velocities_from_positions(
            positioner_mm_float,
            ik_result.sample_rate,
        )

    hastigheter_mm_s = np.round(hastigheter_mm_s_float)
    hastigheter_mm_s = np.clip(
        hastigheter_mm_s,
        -32768,
        32767,
    ).astype(np.int16)

    step = max(1, int(send_every_nth))

    # Packet dt is the time between sent samples.
    dt_ms = int(round(1000.0 / ik_result.sample_rate)) * step
    dt_ms = max(1, dt_ms)

    sample_indices = _compute_sample_indices(len(ik_result.time), step)

    print("Streaming IK to Arduino")
    print(f"Port: {port}")
    print(f"Baudrate: {baudrate}")
    print(f"Neutral leg length: {neutral_mm:.2f} mm")
    print(f"Actuator center: {actuator_center_mm:.1f} mm")
    print(f"Actuator min/max: {actuator_min_mm:.1f} / {actuator_max_mm:.1f} mm")
    print(f"IK sample rate: {ik_result.sample_rate:.2f} Hz")
    print(f"send_every_nth: {send_every_nth}")
    print(f"dt_ms in packet: {dt_ms}")
    print(f"original samples: {len(ik_result.time)}")
    print(f"sent packets: {len(sample_indices)}")
    print(f"batch_size: {batch_size}")
    print(f"wait_between_batches_s: {wait_between_batches_s}")
    print(f"calibrate_before_stream: {calibrate_before_stream}")
    print()

    print("Position range being sent:")
    for i in range(Antal_motorer):
        print(
            f"  Motor {i + 1}: "
            f"{positioner_mm[:, i].min()} to {positioner_mm[:, i].max()} mm"
        )

    print("Velocity range being sent:")
    for i in range(Antal_motorer):
        print(
            f"  Motor {i + 1}: "
            f"{hastigheter_mm_s[:, i].min()} to {hastigheter_mm_s[:, i].max()} mm/s"
        )

    print()

    port = resolve_serial_port(port)

    print(f"Opening serial port {port} @ {baudrate}")
    print(f"Waiting {startup_delay_s:.1f} seconds for Arduino startup...")

    monitor = SerialMonitor(log_file=log_serial)

    sekvens = 0

    with serial.Serial(port, baudrate, timeout=0.1, write_timeout=1.0) as ser:
        time.sleep(startup_delay_s)

        startup_messages = monitor.read_and_process(ser)
        if startup_messages:
            print("\n--- Startup Messages ---")
            monitor.display_messages(startup_messages)
            print()

        if calibrate_before_stream:
            print("Requesting Arduino calibration...")
            ser.write(Kalibrerings_kommando)
            ser.flush()

            print(f"Waiting {calibration_wait_s:.1f} seconds for calibration...")
            time.sleep(calibration_wait_s)

            calibration_messages = monitor.read_and_process(ser)
            if calibration_messages:
                print("\n--- Calibration Messages ---")
                monitor.display_messages(calibration_messages)
                print()

            ser.reset_input_buffer()
            ser.reset_output_buffer()

            print("Calibration wait complete. Starting trajectory stream.\n")

        stream_start = time.perf_counter()
        next_send_time = stream_start

        for batch_start in range(0, len(sample_indices), batch_size):
            batch_indices = sample_indices[batch_start : batch_start + batch_size]
            for sample_index in batch_indices:
                now = time.perf_counter()
                sleep_time = next_send_time - now

                if sleep_time > 0:
                    time.sleep(sleep_time)

                pkt = bygg_paket(
                    sekvens=sekvens,
                    dt_ms=dt_ms,
                    positioner_mm=positioner_mm[sample_index],
                    hastigheter_mm_s=hastigheter_mm_s[sample_index],
                )

                ser.write(pkt)
                ser.flush()

                print(
                    f"Sent seq={sekvens:5d} "
                    f"sample={sample_index:5d} "
                    f"pos={positioner_mm[sample_index].tolist()} "
                    f"vel={hastigheter_mm_s[sample_index].tolist()}"
                )

                incoming_messages = monitor.read_and_process(ser)
                if incoming_messages:
                    monitor.display_messages(incoming_messages)

                sekvens = (sekvens + 1) & 0xFFFF

                next_send_time += dt_ms / 1000.0

            if wait_between_batches_s > 0.0:
                print(f"Batch done. Waiting {wait_between_batches_s:.3f} seconds...")
                time.sleep(wait_between_batches_s)
                next_send_time = time.perf_counter()

        print("\nTrajectory stream complete.")
        print("Reading final messages from Arduino...")

        time.sleep(0.5)

        final_messages = monitor.read_and_process(ser)
        if final_messages:
            print("--- Final Messages ---")
            monitor.display_messages(final_messages)

        print(monitor.get_summary())
