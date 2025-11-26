#include <iostream>
#include <windows.h>
#include <deque>
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <iomanip>

class SerialPort {
private:
    HANDLE hSerial;
    std::string port;
    int baudrate;

public:
    SerialPort(const std::string& port, int baudrate) : port(port), baudrate(baudrate), hSerial(INVALID_HANDLE_VALUE) {}

    bool connect() {
        std::wstring wport(port.begin(), port.end());
        hSerial = CreateFileW(wport.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

        if (hSerial == INVALID_HANDLE_VALUE) {
            std::cout << "포트 연결 실패: " << port << std::endl;
            return false;
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            std::cout << "포트 상태 가져오기 실패" << std::endl;
            CloseHandle(hSerial);
            return false;
        }

        dcbSerialParams.BaudRate = baudrate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            std::cout << "포트 설정 실패" << std::endl;
            CloseHandle(hSerial);
            return false;
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 10;
        timeouts.ReadTotalTimeoutConstant = 10;
        timeouts.ReadTotalTimeoutMultiplier = 1;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            std::cout << "타임아웃 설정 실패" << std::endl;
            CloseHandle(hSerial);
            return false;
        }

        return true;
    }

    int readData(char* buffer, int size) {
        DWORD bytesRead;
        if (ReadFile(hSerial, buffer, size, &bytesRead, NULL)) {
            return bytesRead;
        }
        return 0;
    }

    void close() {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
            hSerial = INVALID_HANDLE_VALUE;
        }
    }

    ~SerialPort() {
        close();
    }
};

class CircularBuffer {
private:
    std::deque<int> buffer;
    size_t maxSize;
    std::mutex mtx;

public:
    CircularBuffer(size_t size) : maxSize(size) {}

    void push(int value) {
        std::lock_guard<std::mutex> lock(mtx);
        buffer.push_back(value);
        if (buffer.size() > maxSize) {
            buffer.pop_front();
        }
    }

    size_t size() {
        std::lock_guard<std::mutex> lock(mtx);
        return buffer.size();
    }

    std::deque<int> getData() {
        std::lock_guard<std::mutex> lock(mtx);
        return buffer;
    }
};

const std::string PORT = "COM6";
const int BAUD = 2000000;
const int BUFFER_SIZE = 1000;

std::atomic<bool> running(true);
std::atomic<int> sample_count(0);
std::atomic<double> data_rate(0.0);
CircularBuffer dataBuffer(BUFFER_SIZE);

void readSerial() {
    SerialPort serial(PORT, BAUD);

    if (!serial.connect()) {
        std::cout << "시리얼 포트 연결 실패" << std::endl;
        return;
    }

    std::cout << "Arduino 연결 완료" << std::endl;

    std::string byteBuffer;
    auto lastTime = std::chrono::steady_clock::now();
    int samplesInPeriod = 0;
    char readBuffer[1024];

    while (running) {
        int bytesRead = serial.readData(readBuffer, sizeof(readBuffer));

        if (bytesRead > 0) {
            byteBuffer.append(readBuffer, bytesRead);

            while (byteBuffer.length() >= 2) {
                unsigned char low = static_cast<unsigned char>(byteBuffer[0]);
                unsigned char high = static_cast<unsigned char>(byteBuffer[1]);
                int value = low + (high << 8);

                if (value >= 0 && value <= 1023) {
                    dataBuffer.push(value);
                    sample_count++;
                    samplesInPeriod++;
                }

                byteBuffer.erase(0, 2);
            }
        }

        auto currentTime = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastTime).count();

        if (elapsed >= 1000) {
            double rate = samplesInPeriod * 1000.0 / elapsed;
            data_rate = rate;
            samplesInPeriod = 0;
            lastTime = currentTime;

            std::cout << "속도: " << std::fixed << std::setprecision(0) << rate
                      << " Hz, 데이터: " << dataBuffer.size() << "개" << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    serial.close();
}

int main() {
    std::cout << "Arduino 연결 중..." << std::endl;

    std::thread serialThread(readSerial);

    std::cout << "데이터 수집 시작. 종료하려면 Enter를 누르세요..." << std::endl;
    std::cin.get();

    running = false;
    serialThread.join();

    std::cout << "종료" << std::endl;
    return 0;
}
