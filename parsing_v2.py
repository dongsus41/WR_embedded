#!/usr/bin/env python3
"""
SMA Wearable Robot Telemetry Parser
COM9에서 텔레메트리 데이터를 읽고 파싱하여 출력

Usage:
    python telemetry_parser.py
    python telemetry_parser.py --port COM3 --baud 115200
"""

import serial
import struct
import time
import argparse
from dataclasses import dataclass
from typing import Optional

# ========== Frame Constants ==========
HEADER_BYTE1 = 0xAA
HEADER_BYTE2 = 0x55
NUM_CHANNELS = 6
NUM_FORCE_SENSORS = 4
NUM_DISPLACEMENT_SENSORS = 5
FRAME_SIZE = 130  # 2 + 4 + 96 + 8 + 10 + 6 + 2 + 2 = 128 bytes

# ========== Data Structures ==========
@dataclass
class ActuatorStatus:
    current_temp: float      # 현재 온도 (°C)
    target_value: float      # 목표값 (온도 또는 힘)
    pwm_duty: float          # PWM Duty (%)
    control_mode: int        # 제어 모드 (0=DISABLED, 1=OPEN, 2=TEMP, 3=FORCE)
    fault_flag: int          # 에러 플래그 (0=정상, 1=과열)

@dataclass
class TelemetryFrame:
    timestamp_ms: int
    actuators: list          # ActuatorStatus x 6
    force_sensors: list      # uint16 x 4
    displacement: list       # uint16 x 5
    fan_duty: list           # uint8 x 6
    system_state: int
    crc16: int
    crc_valid: bool

# ========== Mode Names ==========
MODE_NAMES = {
    0: "DISABLED",
    1: "OPEN_LOOP",
    2: "TEMP_CTRL",
    3: "FORCE_CTRL",
    4: "POS_CTRL"
}

# ========== CRC-16 Calculation ==========
def calculate_crc16(data: bytes) -> int:
    """CRC-16 CCITT 계산"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF
    return crc

# ========== Frame Parser ==========
def parse_actuator_status(data: bytes) -> ActuatorStatus:
    """ActuatorStatus 파싱 (16 bytes)
    
    Layout:
        float current_temp   (4 bytes)
        float target_value   (4 bytes)
        float pwm_duty       (4 bytes)
        uint8 control_mode   (1 byte)
        uint8 fault_flag     (1 byte)
        uint8 reserved[2]    (2 bytes)
    """
    current_temp, target_value, pwm_duty, control_mode, fault_flag = \
        struct.unpack('<fffBB', data[:14])
    # reserved 2 bytes 무시
    return ActuatorStatus(
        current_temp=current_temp,
        target_value=target_value,
        pwm_duty=pwm_duty,
        control_mode=control_mode,
        fault_flag=fault_flag
    )

def parse_telemetry_frame(data: bytes) -> Optional[TelemetryFrame]:
    """전체 텔레메트리 프레임 파싱 (128 bytes)

    Layout:
        header[2]           (2 bytes)  - 0xAA, 0x55
        timestamp_ms        (4 bytes)  - uint32 LE
        actuator[6]         (96 bytes) - 6 x 16 bytes
        force_sensor[4]     (8 bytes)  - 4 x uint16 LE
        displacement[5]     (10 bytes) - 5 x uint16 LE
        fan_duty[6]         (6 bytes)  - 6 x uint8
        system_state        (1 byte)
        reserved            (1 byte)
        crc16               (2 bytes)  - uint16 LE
    """
    try:
        offset = 0
        
        # Header (2 bytes) - 이미 검증됨
        offset += 2
        
        # Timestamp (4 bytes) - Little Endian
        timestamp_ms = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        
        # Actuators (6 x 16 bytes = 96 bytes)
        actuators = []
        for i in range(NUM_CHANNELS):
            act = parse_actuator_status(data[offset:offset+16])
            actuators.append(act)
            offset += 16
        
        # Force sensors (4 x uint16 = 8 bytes)
        # Little Endian으로 읽기 (MCU에서 그대로 저장)
        force_sensors = list(struct.unpack('<4H', data[offset:offset+8]))
        offset += 8

        # Displacement (5 x uint16 = 10 bytes)
        displacement = list(struct.unpack('<5H', data[offset:offset+10]))
        offset += 10
        
        # Fan duty (6 x uint8 = 6 bytes)
        fan_duty = list(struct.unpack('<6B', data[offset:offset+6]))
        offset += 6
        
        # System state (1 byte)
        system_state = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
        
        # Reserved (1 byte)
        offset += 1
        
        # CRC16 (2 bytes)
        crc16 = struct.unpack('<H', data[offset:offset+2])[0]

        # CRC 검증 (CRC 필드 제외한 128 bytes)
        crc_calculated = calculate_crc16(data[:-2])
        crc_valid = (crc16 == crc_calculated)
        
        return TelemetryFrame(
            timestamp_ms=timestamp_ms,
            actuators=actuators,
            force_sensors=force_sensors,
            displacement=displacement,
            fan_duty=fan_duty,
            system_state=system_state,
            crc16=crc16,
            crc_valid=crc_valid
        )
    except Exception as e:
        print(f"Parse error: {e}")
        return None

# ========== Display ==========
def print_telemetry(frame: TelemetryFrame, clear_screen: bool = True):
    """텔레메트리 출력"""
    if clear_screen:
        print("\033[2J\033[H", end="")  # Clear screen
    
    crc_status = "OK" if frame.crc_valid else "FAIL"
    print(f"====================================================================")
    print(f"  SMA Wearable Robot Telemetry  |  Time: {frame.timestamp_ms:>10} ms  |  CRC: {crc_status}")
    print(f"====================================================================")
    
    # Actuator table header
    print(f"  CH |   Temp   |  Target  |   PWM   |    Mode    | Fault | Fan")
    print(f"-----+----------+----------+---------+------------+-------+-----")
    
    for i, act in enumerate(frame.actuators):
        mode_name = MODE_NAMES.get(act.control_mode, "UNKNOWN")
        fault_str = " HOT " if act.fault_flag else " OK  "
        fan = frame.fan_duty[i]
        
        print(f"  {i}  | {act.current_temp:7.2f}C | {act.target_value:8.2f} | {act.pwm_duty:6.1f}% | {mode_name:^10} | {fault_str} | {fan:3d}%")
    
    print(f"-----+----------+----------+---------+------------+-------+-----")
    
    # Force sensors & Displacement
    print(f"  Force Sensors : F0={frame.force_sensors[0]:5d}  F1={frame.force_sensors[1]:5d}  F2={frame.force_sensors[2]:5d}  F3={frame.force_sensors[3]:5d}")
    print(f"  Displacement  : D0={frame.displacement[0]:5d}  D1={frame.displacement[1]:5d}  D2={frame.displacement[2]:5d}  D3={frame.displacement[3]:5d}  D4={frame.displacement[4]:5d}")
    print(f"  System State  : {frame.system_state}")
    print(f"====================================================================")

def print_telemetry_compact(frame: TelemetryFrame):
    """컴팩트 출력 (로그용)"""
    temps = [f"{a.current_temp:.1f}" for a in frame.actuators]
    forces = [str(f) for f in frame.force_sensors]
    crc_status = "OK" if frame.crc_valid else "ERR"
    
    print(f"[{frame.timestamp_ms:>10}ms] T:[{','.join(temps)}] F:[{','.join(forces)}] CRC:{crc_status}")

# ========== Main ==========
def main():
    parser = argparse.ArgumentParser(description='SMA Telemetry Parser')
    parser.add_argument('--port', type=str, default='COM9', help='Serial port (default: COM9)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--raw', action='store_true', help='Show raw hex data')
    parser.add_argument('--no-clear', action='store_true', help='Do not clear screen (log mode)')
    parser.add_argument('--compact', action='store_true', help='Compact single-line output')
    args = parser.parse_args()
    
    print(f"Connecting to {args.port} at {args.baud} baud...")
    print(f"Frame size: {FRAME_SIZE} bytes")
    print("Press Ctrl+C to exit\n")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
        print(f"Connected! Waiting for data...\n")
        
        buffer = bytearray()
        frame_count = 0
        error_count = 0
        crc_error_count = 0
        
        while True:
            # 데이터 읽기
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
            
            # 헤더 찾기 (동기화)
            while len(buffer) >= 2:
                if buffer[0] == HEADER_BYTE1 and buffer[1] == HEADER_BYTE2:
                    break
                buffer.pop(0)  # 헤더 아니면 버림
            
            # 프레임 완성 체크
            if len(buffer) >= FRAME_SIZE:
                frame_data = bytes(buffer[:FRAME_SIZE])
                buffer = buffer[FRAME_SIZE:]
                
                if args.raw:
                    print(f"RAW [{len(frame_data)}]: {frame_data.hex()}")
                
                frame = parse_telemetry_frame(frame_data)
                if frame:
                    frame_count += 1
                    if not frame.crc_valid:
                        crc_error_count += 1
                    
                    if args.compact:
                        print_telemetry_compact(frame)
                    else:
                        print_telemetry(frame, clear_screen=not args.no_clear)
                        print(f"  Frames: {frame_count} | Parse Errors: {error_count} | CRC Errors: {crc_error_count}")
                else:
                    error_count += 1
            
            time.sleep(0.001)  # CPU 사용률 감소
            
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("\n\nExiting...")
        print(f"Total: {frame_count} frames, {error_count} parse errors, {crc_error_count} CRC errors")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()