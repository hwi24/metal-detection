import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import time


# 설정
PORT = 'COM6'  # Arduino 포트
BAUD = 2000000
BUFFER_SIZE = 1000

# 데이터 저장
data = deque(maxlen=BUFFER_SIZE)
sample_count = 0
data_rate = 0
running = True

# 시리얼 연결
print("Arduino 연결 중...")
ser = serial.Serial(PORT, BAUD, timeout=0.01)
time.sleep(2)
print("연결 완료")

def read_serial():
    """바이너리 데이터 읽기"""
    global sample_count, data_rate
    
    byte_buffer = b''
    last_time = time.time()
    samples_in_period = 0
    
    while running:
        if ser.in_waiting > 0:
            new_bytes = ser.read(ser.in_waiting)
            byte_buffer += new_bytes
            
            # 2바이트씩 처리
            while len(byte_buffer) >= 2:
                low = byte_buffer[0]
                high = byte_buffer[1]
                value = low + (high << 8)
                
                # ADC 범위 확인
                if 0 <= value <= 1023:
                    data.append(value)
                    sample_count += 1
                    samples_in_period += 1
                
                byte_buffer = byte_buffer[2:]
        
        # 속도 계산 (1초마다)
        current_time = time.time()
        if current_time - last_time >= 1.0:
            data_rate = samples_in_period / (current_time - last_time)
            samples_in_period = 0
            last_time = current_time
            print(f"속도: {data_rate:.0f} Hz, 데이터: {len(data)}개")

# 그래프 설정
fig, ax = plt.subplots(figsize=(12, 6))
line, = ax.plot([], [], 'b-', linewidth=1)
ax.set_ylim(0, 1024)
ax.set_ylabel('ADC Value')
ax.set_xlabel('Sample')
ax.set_title('Arduino ADC 실시간 데이터')
ax.grid(True, alpha=0.3)

def animate(frame):
    """그래프 업데이트"""
    if len(data) > 1:
        x = list(range(len(data)))
        y = list(data)
        line.set_data(x, y)
        ax.set_xlim(0, len(data))
        ax.set_title(f'Arduino ADC - {data_rate:.0f} Hz')
    return line,

# 시리얼 읽기 스레드 시작
thread = threading.Thread(target=read_serial, daemon=True)
thread.start()

# 애니메이션 시작
ani = animation.FuncAnimation(fig, animate, interval=100, blit=True)

print("그래프 창 표시 중...")
try:
    plt.show()
except KeyboardInterrupt:
    pass

# 종료
running = False
ser.close()
print("종료")