#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <deque>
#include <mutex>

// Windows용 시리얼 통신
#ifdef _WIN32
#include <windows.h>
#include <locale>
#include <codecvt>
#else
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#endif

// SFML for 실시간 그래프 (cross-platform)
#include <SFML/Graphics.hpp>

class SerialReader {
private:
#ifdef _WIN32
    HANDLE hSerial;
#else
    int serialfd;
#endif
    
    std::atomic<bool> running{true};
    std::atomic<int> sampleCount{0};
    std::atomic<double> dataRate{0.0};
    
    std::deque<uint16_t> dataBuffer;
    std::mutex dataMutex;
    static const size_t MAX_BUFFER_SIZE = 1000;
    
public:
    SerialReader(const std::string& port, int baudRate) {
#ifdef _WIN32
        // Windows 시리얼 포트 초기화
        hSerial = CreateFileA(
            port.c_str(),
            GENERIC_READ | GENERIC_WRITE,
            0,
            NULL,
            OPEN_EXISTING,
            FILE_ATTRIBUTE_NORMAL,
            NULL
        );
        
        if (hSerial == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("시리얼 포트 열기 실패: " + port);
        }
        
        // 시리얼 포트 설정
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("시리얼 상태 가져오기 실패");
        }
        
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("시리얼 설정 실패");
        }
        
        // 타임아웃 설정 (논블로킹)
        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = MAXDWORD;
        timeouts.ReadTotalTimeoutConstant = 0;
        timeouts.ReadTotalTimeoutMultiplier = 0;
        SetCommTimeouts(hSerial, &timeouts);
        
#else
        // Linux/Mac 시리얼 포트 초기화
        serialfd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serialfd == -1) {
            throw std::runtime_error("시리얼 포트 열기 실패: " + port);
        }
        
        struct termios options;
        tcgetattr(serialfd, &options);
        
        // 보드레이트 설정 (2M baud는 B2000000)
        cfsetispeed(&options, B2000000);
        cfsetospeed(&options, B2000000);
        
        // 8N1 설정
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        
        tcsetattr(serialfd, TCSANOW, &options);
#endif
        
        std::cout << "시리얼 포트 연결 완료: " << port << std::endl;
    }
    
    ~SerialReader() {
        running = false;
#ifdef _WIN32
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
        }
#else
        if (serialfd != -1) {
            close(serialfd);
        }
#endif
    }
    
    void startReading() {
        std::thread readerThread(&SerialReader::readLoop, this);
        readerThread.detach();
    }
    
    void stop() {
        running = false;
    }
    
    std::vector<uint16_t> getData() {
        std::lock_guard<std::mutex> lock(dataMutex);
        return std::vector<uint16_t>(dataBuffer.begin(), dataBuffer.end());
    }
    
    double getDataRate() const {
        return dataRate.load();
    }
    
private:
    void readLoop() {
        std::vector<uint8_t> byteBuffer;
        auto lastTime = std::chrono::steady_clock::now();
        int samplesInPeriod = 0;
        
        while (running) {
            // 시리얼 데이터 읽기
            uint8_t buffer[1024];
            int bytesRead = 0;
            
#ifdef _WIN32
            DWORD dwBytesRead;
            if (ReadFile(hSerial, buffer, sizeof(buffer), &dwBytesRead, NULL)) {
                bytesRead = dwBytesRead;
            }
#else
            bytesRead = read(serialfd, buffer, sizeof(buffer));
            if (bytesRead < 0) bytesRead = 0;
#endif
            
            if (bytesRead > 0) {
                // 바이트 버퍼에 추가
                for (int i = 0; i < bytesRead; i++) {
                    byteBuffer.push_back(buffer[i]);
                }
                
                // 2바이트씩 처리
                while (byteBuffer.size() >= 2) {
                    uint16_t value = byteBuffer[0] | (byteBuffer[1] << 8);
                    
                    // ADC 범위 확인
                    if (value <= 1023) {
                        std::lock_guard<std::mutex> lock(dataMutex);
                        dataBuffer.push_back(value);
                        if (dataBuffer.size() > MAX_BUFFER_SIZE) {
                            dataBuffer.pop_front();
                        }
                        
                        sampleCount++;
                        samplesInPeriod++;
                    }
                    
                    // 처리된 바이트 제거
                    byteBuffer.erase(byteBuffer.begin(), byteBuffer.begin() + 2);
                }
            }
            
            // 속도 계산 (1초마다)
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTime - lastTime).count();
            
            if (elapsed >= 1000) {
                double rate = (double)samplesInPeriod * 1000.0 / elapsed;
                dataRate = rate;
                samplesInPeriod = 0;
                lastTime = currentTime;
                
                std::cout << "속도: " << (int)rate << " Hz, 데이터: " 
                         << dataBuffer.size() << "개" << std::endl;
            }
            
            // CPU 부하 감소
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
};

class RealtimePlotter {
private:
    sf::RenderWindow window;
    SerialReader& serialReader;
    sf::Font font;
    
    // 그래프 설정
    static const int WINDOW_WIDTH = 1200;
    static const int WINDOW_HEIGHT = 600;
    static const int GRAPH_MARGIN = 50;
    
public:
    RealtimePlotter(SerialReader& reader) 
        : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Arduino ADC 실시간 데이터")
        , serialReader(reader) {
        
        window.setFramerateLimit(60);
        
        // 폰트 로드 (시스템 기본 폰트 사용)
        if (!font.loadFromFile("C:/Windows/Fonts/arial.ttf")) {
            // Windows가 아니거나 폰트 로드 실패시 기본값 사용
            std::cout << "폰트 로드 실패, 기본 폰트 사용" << std::endl;
        }
    }
    
    void run() {
        while (window.isOpen()) {
            handleEvents();
            render();
        }
        serialReader.stop();
    }
    
private:
    void handleEvents() {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
    }
    
    void render() {
        window.clear(sf::Color::Black);
        
        // 데이터 가져오기
        auto data = serialReader.getData();
        
        if (data.size() > 1) {
            // 그래프 영역 계산
            float graphWidth = WINDOW_WIDTH - 2 * GRAPH_MARGIN;
            float graphHeight = WINDOW_HEIGHT - 2 * GRAPH_MARGIN;
            
            // 데이터 포인트들을 선으로 연결
            sf::VertexArray lines(sf::LineStrip, data.size());
            
            for (size_t i = 0; i < data.size(); i++) {
                float x = GRAPH_MARGIN + (float)i * graphWidth / (data.size() - 1);
                float y = WINDOW_HEIGHT - GRAPH_MARGIN - 
                         (float)data[i] * graphHeight / 1024.0f;
                
                lines[i] = sf::Vertex(sf::Vector2f(x, y), sf::Color::Green);
            }
            
            window.draw(lines);
            
            // 격자 그리기
            drawGrid();
            
            // 제목과 정보 표시
            drawInfo();
        }
        
        window.display();
    }
    
    void drawGrid() {
        sf::VertexArray grid(sf::Lines);
        
        // 세로 격자선
        for (int i = 0; i <= 10; i++) {
            float x = GRAPH_MARGIN + i * (WINDOW_WIDTH - 2 * GRAPH_MARGIN) / 10.0f;
            grid.append(sf::Vertex(sf::Vector2f(x, GRAPH_MARGIN), sf::Color(50, 50, 50)));
            grid.append(sf::Vertex(sf::Vector2f(x, WINDOW_HEIGHT - GRAPH_MARGIN), sf::Color(50, 50, 50)));
        }
        
        // 가로 격자선
        for (int i = 0; i <= 8; i++) {
            float y = GRAPH_MARGIN + i * (WINDOW_HEIGHT - 2 * GRAPH_MARGIN) / 8.0f;
            grid.append(sf::Vertex(sf::Vector2f(GRAPH_MARGIN, y), sf::Color(50, 50, 50)));
            grid.append(sf::Vertex(sf::Vector2f(WINDOW_WIDTH - GRAPH_MARGIN, y), sf::Color(50, 50, 50)));
        }
        
        window.draw(grid);
    }
    
    void drawInfo() {
        sf::Text title;
        title.setFont(font);
        title.setString("Arduino ADC 실시간 데이터 - " + 
                       std::to_string((int)serialReader.getDataRate()) + " Hz");
        title.setCharacterSize(24);
        title.setFillColor(sf::Color::White);
        title.setPosition(10, 10);
        
        window.draw(title);
        
        // Y축 레이블
        for (int i = 0; i <= 8; i++) {
            int value = 1024 - (1024 * i / 8);
            float y = GRAPH_MARGIN + i * (WINDOW_HEIGHT - 2 * GRAPH_MARGIN) / 8.0f;
            
            sf::Text label;
            label.setFont(font);
            label.setString(std::to_string(value));
            label.setCharacterSize(12);
            label.setFillColor(sf::Color::White);
            label.setPosition(10, y - 6);
            
            window.draw(label);
        }
    }
};

int main() {
    try {
        std::cout << "Arduino 연결 중..." << std::endl;
        
        // COM 포트는 시스템에 맞게 변경 필요
        std::string port;
#ifdef _WIN32
        port = "COM6";
#else
        port = "/dev/ttyACM0";  // Linux/Mac
#endif
        
        SerialReader reader(port, 2000000);
        reader.startReading();
        
        // 잠시 대기 (Arduino 초기화)
        std::this_thread::sleep_for(std::chrono::seconds(2));
        
        std::cout << "그래프 창 표시 중..." << std::endl;
        RealtimePlotter plotter(reader);
        plotter.run();
        
    } catch (const std::exception& e) {
        std::cerr << "오류: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "프로그램 종료" << std::endl;
    return 0;
}

// 컴파일 방법 (Windows + SFML):
// g++ -std=c++17 arduino_plotter.cpp -lsfml-graphics -lsfml-window -lsfml-system -o arduino_plotter.exe

// 컴파일 방법 (Linux + SFML):
// g++ -std=c++17 arduino_plotter.cpp -lsfml-graphics -lsfml-window -lsfml-system -o arduino_plotter