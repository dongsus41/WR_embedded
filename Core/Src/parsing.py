import can

def parse_can_msg(msg):
    if msg.arbitration_id == 0x401:
        data = msg.data
        
        # 1. PWM 출력 (처음 6바이트)
        pwm_outputs = list(data[0:6])
        
        # 2. Fan 상태 (다음 6바이트)
        fan_states = list(data[6:12])
        
        # 3. 온도 (2바이트씩 6채널, Little Endian)
        temps = []
        for i in range(6):
            start = 12 + (i * 2)
            # 리틀 엔디언: 뒤의 바이트가 상위(High), 앞이 하위(Low)
            raw_temp = data[start] | (data[start+1] << 8)
            temps.append(raw_temp / 4)
            
        print(f"\n[ID 401 현재 상태]")
        print(f"PWM 출력: {pwm_outputs}")
        print(f"Fan 상태: {fan_states}")
        print(f"채널 온도: {[f'{t}℃' for t in temps]}")

# SocketCAN 인터페이스 설정
bus = can.interface.Bus(channel='can0', bustype='socketcan')

print("데이터 수신 대기 중... (Ctrl+C로 종료)")
try:
    while True:
        message = bus.recv()  # 메시지 수신 대기
        if message is not None:
            parse_can_msg(message)
except KeyboardInterrupt:
    print("\n종료합니다.")