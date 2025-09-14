#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <string>
#include <deque>
#include <mutex>
#include <io.h>
#include <cstdlib>
#include <cmath>

// Windows용 시리얼 통신
#ifdef _WIN32
#include <windows.h>
#include <locale>
#include <codecvt>
#include <fcntl.h>
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
            throw std::runtime_error("Failed to open serial port: " + port);
        }
        
        // 시리얼 포트 설정
        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
        
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Failed to get serial state");
        }
        
        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;
        
        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Failed to set serial configuration");
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
            throw std::runtime_error("Failed to open serial port: " + port);
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
        
        std::cout << "Serial port connected: " << port << std::endl;
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
                
                std::cout << "Rate: " << (int)rate << " Hz, Data: " 
                         << dataBuffer.size() << " samples" << std::endl;
            }
            
            // CPU 부하 감소
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }
};

class SimulationReader {
private:
    std::atomic<bool> running{true};
    std::atomic<int> sampleCount{0};
    std::atomic<double> dataRate{0.0};

    std::deque<uint16_t> dataBuffer;
    std::mutex dataMutex;
    static const size_t MAX_BUFFER_SIZE = 1000;

public:
    SimulationReader() {
        std::cout << "Simulation mode initialized" << std::endl;
    }

    ~SimulationReader() {
        running = false;
    }

    void startReading() {
        std::thread readerThread(&SimulationReader::simulateLoop, this);
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
    void simulateLoop() {
        auto lastTime = std::chrono::steady_clock::now();
        int samplesInPeriod = 0;
        double time = 0.0;

        while (running) {
            // Generate simulated ADC data (sine wave with some noise)
            double sineValue = 512 + 300 * std::sin(time * 2 * 3.14159 / 100.0); // Slow sine wave
            double noise = (rand() % 100 - 50) * 0.5; // Random noise
            uint16_t value = std::max(0.0, std::min(1023.0, sineValue + noise));

            {
                std::lock_guard<std::mutex> lock(dataMutex);
                dataBuffer.push_back(value);
                if (dataBuffer.size() > MAX_BUFFER_SIZE) {
                    dataBuffer.pop_front();
                }
            }

            sampleCount++;
            samplesInPeriod++;
            time += 1.0;

            // Calculate rate (every second)
            auto currentTime = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                currentTime - lastTime).count();

            if (elapsed >= 1000) {
                double rate = (double)samplesInPeriod * 1000.0 / elapsed;
                dataRate = rate;
                samplesInPeriod = 0;
                lastTime = currentTime;

                std::cout << "Simulation Rate: " << (int)rate << " Hz, Data: "
                         << dataBuffer.size() << " samples" << std::endl;
            }

            // Simulate ~100 Hz data rate
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
};

template<typename DataReader>
class RealtimePlotter {
private:
    sf::RenderWindow window;
    DataReader& dataReader;
    sf::Font font;
    
    // 그래프 설정
    static const int WINDOW_WIDTH = 1200;
    static const int WINDOW_HEIGHT = 600;
    static const int GRAPH_MARGIN = 50;
    
public:
    RealtimePlotter(DataReader& reader)
        : window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Arduino ADC Real-time Data")
        , dataReader(reader) {
        
        window.setFramerateLimit(60);
        
        // Load system font (cross-platform)
        bool fontLoaded = false;
#ifdef _WIN32
        // Try common Windows font paths
        std::vector<std::string> windowsFonts = {
            "C:/Windows/Fonts/arial.ttf",
            "C:/Windows/Fonts/calibri.ttf", 
            "C:/Windows/Fonts/tahoma.ttf"
        };
        for (const auto& fontPath : windowsFonts) {
            if (font.loadFromFile(fontPath)) {
                fontLoaded = true;
                break;
            }
        }
#else
        // Try common Linux/Mac font paths
        std::vector<std::string> unixFonts = {
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/TTF/arial.ttf",
            "/System/Library/Fonts/Arial.ttf",  // macOS
            "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf"
        };
        for (const auto& fontPath : unixFonts) {
            if (font.loadFromFile(fontPath)) {
                fontLoaded = true;
                break;
            }
        }
#endif
        if (!fontLoaded) {
            std::cout << "Font load failed, using default font" << std::endl;
        }
    }
    
    void run() {
        while (window.isOpen()) {
            handleEvents();
            render();
        }
        dataReader.stop();
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
        auto data = dataReader.getData();
        
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
        title.setString("Arduino ADC Real-time Data - " +
                       std::to_string((int)dataReader.getDataRate()) + " Hz");
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
#ifdef _WIN32
        // Windows 콘솔 UTF-8 인코딩 설정
        SetConsoleOutputCP(CP_UTF8);
        SetConsoleCP(CP_UTF8);
#endif
        std::cout << "Arduino Real-time Plotter" << std::endl;
        std::cout << "1. Connect to Arduino" << std::endl;
        std::cout << "2. Simulation mode (demo)" << std::endl;
        std::cout << "Choose option (1 or 2): ";

        std::string choice;
        std::getline(std::cin, choice);

        if (choice == "2") {
            // Simulation mode
            std::cout << "Starting simulation mode..." << std::endl;
            SimulationReader simReader;
            simReader.startReading();

            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::cout << "Displaying simulation window..." << std::endl;
            RealtimePlotter plotter(simReader);
            plotter.run();
        } else {
            // Arduino mode
            std::cout << "Arduino connecting..." << std::endl;

            // Get serial port from user input
            std::string port;
            std::cout << "Enter serial port (e.g., ";
#ifdef _WIN32
            std::cout << "COM6): ";
#else
            std::cout << "/dev/ttyACM0): ";
#endif
            std::getline(std::cin, port);

            // Use default if empty
            if (port.empty()) {
#ifdef _WIN32
                port = "COM6";
#else
                port = "/dev/ttyACM0";
#endif
                std::cout << "Using default port: " << port << std::endl;
            }

            SerialReader reader(port, 2000000);
            reader.startReading();

            // Wait for Arduino initialization
            std::this_thread::sleep_for(std::chrono::seconds(2));

            std::cout << "Displaying graph window..." << std::endl;
            RealtimePlotter plotter(reader);
            plotter.run();
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    }
    
    std::cout << "Program terminated" << std::endl;
    return 0;
}

// 컴파일 방법 (Windows + SFML):
// g++ -std=c++17 arduino_plotter.cpp -lsfml-graphics -lsfml-window -lsfml-system -o arduino_plotter.exe

// 컴파일 방법 (Linux + SFML):
// g++ -std=c++17 arduino_plotter.cpp -lsfml-graphics -lsfml-window -lsfml-system -o arduino_plotter