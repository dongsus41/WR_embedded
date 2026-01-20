#!/usr/bin/env python3
"""
SMA Wearable Robot Telemetry Parser v3 (Interactive TUI)
기능:
  1. 실시간 텔레메트리 파싱 및 고정된 화면 출력 (curses 사용)
  2. 하단 프롬프트를 통한 시리얼 명령어 전송 기능
  3. 수신(Read)과 송신/UI(Write) 스레드 분리

Usage:
    python telemetry_parser_v3.py --port /dev/ttyUSB0 --baud 115200
"""

import serial
import struct
import time
import argparse
import threading
import curses
import queue
from dataclasses import dataclass
from typing import Optional

# ========== Frame Constants ==========
HEADER_BYTE1 = 0xAA
HEADER_BYTE2 = 0x55
NUM_CHANNELS = 6
FRAME_SIZE = 130 

# ========== Data Structures ==========
@dataclass
class ActuatorStatus:
    current_temp: float      
    target_value: float      
    pwm_duty: float          
    control_mode: int        
    fault_flag: int          

@dataclass
class TelemetryFrame:
    timestamp_ms: int
    actuators: list          
    force_sensors: list      
    displacement: list       
    fan_duty: list           
    system_state: int
    crc16: int
    crc_valid: bool

MODE_NAMES = {
    0: "DISABLE", 1: "OPEN", 2: "TEMP", 3: "FORCE", 4: "POS"
}

# ========== Shared Resources ==========
# 스레드 간 데이터 공유를 위한 변수들
latest_frame: Optional[TelemetryFrame] = None
data_lock = threading.Lock()
command_queue = queue.Queue()
running = True  # 프로그램 종료 플래그

# ========== Logic Functions (기존 유지) ==========
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000: crc = (crc << 1) ^ 0x1021
            else: crc = crc << 1
            crc &= 0xFFFF
    return crc

def parse_actuator_status(data: bytes) -> ActuatorStatus:
    current_temp, target_value, pwm_duty, control_mode, fault_flag = \
        struct.unpack('<fffBB', data[:14])
    return ActuatorStatus(current_temp, target_value, pwm_duty, control_mode, fault_flag)

def parse_telemetry_frame(data: bytes) -> Optional[TelemetryFrame]:
    try:
        offset = 2 # Header skipped
        timestamp_ms = struct.unpack('<I', data[offset:offset+4])[0]
        offset += 4
        
        actuators = []
        for i in range(NUM_CHANNELS):
            act = parse_actuator_status(data[offset:offset+16])
            actuators.append(act)
            offset += 16
        
        force_sensors = list(struct.unpack('<4H', data[offset:offset+8]))
        offset += 8
        displacement = list(struct.unpack('<5H', data[offset:offset+10]))
        offset += 10
        fan_duty = list(struct.unpack('<6B', data[offset:offset+6]))
        offset += 6
        system_state = struct.unpack('<B', data[offset:offset+1])[0]
        offset += 1
        offset += 1 # Reserved
        crc16 = struct.unpack('<H', data[offset:offset+2])[0]

        crc_calculated = calculate_crc16(data[:-2])
        crc_valid = (crc16 == crc_calculated)
        
        return TelemetryFrame(
            timestamp_ms, actuators, force_sensors, displacement, 
            fan_duty, system_state, crc16, crc_valid
        )
    except Exception:
        return None

# ========== Serial Thread ==========
def serial_worker(port, baud):
    """백그라운드에서 시리얼 데이터를 계속 읽고, 명령어가 있으면 전송함"""
    global latest_frame, running
    
    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        # UI 스레드에서 에러를 표시할 수 있도록 함 (여기서는 간단히 종료 처리)
        running = False
        return

    buffer = bytearray()
    
    while running:
        # 1. Write Commands (if any)
        while not command_queue.empty():
            cmd = command_queue.get()
            if cmd:
                # 명령어 끝에 개행문자 추가하여 전송 (필요시 프로토콜에 맞게 수정)
                full_cmd = f"{cmd}\n".encode('utf-8')
                ser.write(full_cmd)
        
        # 2. Read Data
        try:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))
                
                # 헤더 찾기 및 파싱 루프
                while len(buffer) >= FRAME_SIZE:
                    # 헤더 확인 (0xAA 0x55)
                    if buffer[0] != HEADER_BYTE1 or buffer[1] != HEADER_BYTE2:
                        buffer.pop(0)
                        continue
                        
                    # 프레임 추출
                    frame_data = bytes(buffer[:FRAME_SIZE])
                    buffer = buffer[FRAME_SIZE:] # 버퍼에서 제거
                    
                    parsed = parse_telemetry_frame(frame_data)
                    if parsed:
                        with data_lock:
                            latest_frame = parsed
            else:
                time.sleep(0.01) # CPU 점유율 방지
        except Exception:
            pass # 읽기 에러 무시 혹은 로깅

    ser.close()

# ========== Curses UI ==========
def draw_ui(stdscr):
    """화면을 그리는 메인 루프"""
    global running, latest_frame
    
    # 설정
    curses.curs_set(1)  # 커서 보이기
    stdscr.nodelay(True) # 키 입력 비차단(Non-blocking) 설정
    stdscr.timeout(100)  # getch() 대기 시간 100ms (화면 갱신 주기)

    # 색상 설정 (지원되는 경우)
    if curses.has_colors():
        curses.start_color()
        curses.init_pair(1, curses.COLOR_CYAN, curses.COLOR_BLACK) # 헤더
        curses.init_pair(2, curses.COLOR_RED, curses.COLOR_BLACK)  # 에러/Hot
        curses.init_pair(3, curses.COLOR_GREEN, curses.COLOR_BLACK) # 정상

    input_buffer = ""
    command_log = [] # 최근 전송한 명령어 기록

    while running:
        stdscr.erase()
        height, width = stdscr.getmaxyx()

        # ------------------
        # 1. 상단: 텔레메트리 출력 (고정 영역)
        # ------------------
        with data_lock:
            frame = latest_frame

        if frame:
            crc_str = "OK" if frame.crc_valid else "FAIL"
            color_crc = curses.color_pair(3) if frame.crc_valid else curses.color_pair(2)
            
            stdscr.addstr(0, 0, f"SMA Robot Telemetry | Time: {frame.timestamp_ms} ms | CRC: ", curses.color_pair(1))
            stdscr.addstr(f"{crc_str}", color_crc)
            
            # Table Header
            header = " CH |  Temp(C) | Target |  PWM(%) |   Mode   | Fault | Fan"
            stdscr.addstr(2, 0, header, curses.A_BOLD)
            stdscr.addstr(3, 0, "-" * len(header))

            # Table Rows
            for i, act in enumerate(frame.actuators):
                mode_str = MODE_NAMES.get(act.control_mode, "UNKN")
                fault_str = "HOT" if act.fault_flag else "OK"
                fault_color = curses.color_pair(2) if act.fault_flag else curses.color_pair(3)
                
                row_str = f" {i}  | {act.current_temp:8.2f} | {act.target_value:6.1f} | {act.pwm_duty:7.1f} | {mode_str:^8} | "
                stdscr.addstr(4 + i, 0, row_str)
                stdscr.addstr(fault_str, fault_color)
                stdscr.addstr(f" | {frame.fan_duty[i]:3d}%")

            # Sensors
            y_off = 4 + NUM_CHANNELS + 1
            stdscr.addstr(y_off, 0, f"Force Sensors: {frame.force_sensors}")
            stdscr.addstr(y_off + 1, 0, f"Displacement : {frame.displacement}")
            stdscr.addstr(y_off + 2, 0, f"System State : {frame.system_state}")

        else:
            stdscr.addstr(0, 0, "Waiting for data...", curses.color_pair(2))

        # ------------------
        # 2. 중간: 로그 영역 (최근 명령어)
        # ------------------
        log_start_y = height - 5
        stdscr.addstr(log_start_y - 1, 0, "-" * width)
        for idx, log in enumerate(command_log[-3:]): # 최근 3개만 표시
            stdscr.addstr(log_start_y + idx, 0, f"Sent: {log}")

        # ------------------
        # 3. 하단: 명령어 입력창
        # ------------------
        prompt = "CMD >> "
        stdscr.addstr(height - 1, 0, prompt, curses.color_pair(1))
        stdscr.addstr(height - 1, len(prompt), input_buffer)

        # 화면 갱신
        stdscr.refresh()

        # ------------------
        # 4. 키보드 입력 처리
        # ------------------
        try:
            key = stdscr.getch()
            if key != -1:
                if key == 10: # Enter Key
                    if input_buffer.strip():
                        command_queue.put(input_buffer) # 큐에 명령어 넣기
                        command_log.append(input_buffer)
                        input_buffer = ""
                elif key == 27: # ESC Key -> 종료
                    running = False
                elif key in (127, 8, curses.KEY_BACKSPACE): # Backspace
                    input_buffer = input_buffer[:-1]
                elif 32 <= key <= 126: # Printable chars
                    input_buffer += chr(key)
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()

    print(f"Connecting to {args.port}...")
    
    # 1. 시리얼 통신 스레드 시작
    t_serial = threading.Thread(target=serial_worker, args=(args.port, args.baud))
    t_serial.daemon = True
    t_serial.start()

    # 2. UI 시작 (Main Thread)
    try:
        curses.wrapper(draw_ui)
    except KeyboardInterrupt:
        pass
    finally:
        global running
        running = False
        t_serial.join(timeout=1)
        print("Exited.")

if __name__ == "__main__":
    main()