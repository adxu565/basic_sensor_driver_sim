#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdint>
#include <chrono>
#include <thread>
#include <random>
#include <iomanip>

class DummyIMU {
    public:
        DummyIMU(const std::string& port, 
                int output_rate): 
            _port_name(port), 
            _serial_port(-1), 
            _output_rate(output_rate),
            _seed(std::random_device{}()) {}
        ~DummyIMU();

        static uint16_t calculateChecksum(uint16_t a, uint16_t b, uint16_t c);
        bool openPort();
        bool closePort();
        void run(ssize_t duration_s = -1);
        
    private:
        std::string _port_name;
        int _serial_port;
        int _output_rate;
        std::mt19937 _seed;
        struct termios _options;

        bool _configureSerialPort();
        ssize_t _writeData(const uint8_t* data, size_t size);
        ssize_t _readData(uint8_t* buffer, size_t size);
};

DummyIMU::~DummyIMU(){
    closePort();  // make sure we disconnect from the port
}

uint16_t DummyIMU::calculateChecksum(uint16_t a, uint16_t b, uint16_t c){
    uint32_t sum = a + b + c;
    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += sum >> 16;
    return static_cast<uint16_t>(~sum);
}

bool DummyIMU::openPort() {
    _serial_port = open(_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_serial_port == -1) {
        perror("ERROR: failed to open port!");
        return false;
    }
    // make sure no other app has this port open
    if (ioctl(_serial_port, TIOCEXCL) == -1) {
        perror("Failed to set exclusive mode, port is already occupied!");
        closePort();
        return false;
    }
    return _configureSerialPort();
}

bool DummyIMU::closePort() {
    if (_serial_port == -1){
        std::cerr << "WARNING: IMU already closed the port!" << std::endl;
        // error handling
        return false;
    }
    int result = close(_serial_port);
    if (result == -1){
        perror("ERROR: failed to close serial port!");
        return false;
    }
    _serial_port = -1;
    std::cout << "Successfully closed port: " << _port_name << std::endl;
    return true;
}

void DummyIMU::run(ssize_t duration_s) {
    std::uniform_int_distribution<> dist{0, 65535};
    int us_period = 1000000 / _output_rate;
    uint16_t data[4];
    bool data_requested = false;
    bool loop_condition = true;
    auto start = std::chrono::steady_clock::now();
    std::chrono::seconds duration(duration_s);
    while(loop_condition){
        // Continuously output random values for x,y,z acceleration
        // Ideally the sensor would be modeled to be realistic,
        // outputting values in response to some dynamics simulation.
        // For temporary simulation, provide duration_s

        // vaguely modeled after the ADXL345, which uses 2 bytes per field
        data[0] = dist(_seed); // accel_x
        data[1] = dist(_seed); // accel_y
        data[2] = dist(_seed); // accel_z
        data[3] = calculateChecksum(data[0], data[1], data[2]); // checksum
        
        // simulate a scenario where checksum is sometimes corrupted
        if (dist(_seed)%5 == 0){
            data[3] = 0;
        }

        // only write data when a driver notifies the IMU that it's listening.
        // for simplicity we can say that whenever it receives 0xFF, it will start
        // or stop writing to the port (idle by default).
        // This is a basic representation of how the driver can send commands to a sensor
        // Read data from the serial port
        uint8_t command[1] = {0};
        ssize_t bytes_read = _readData(command, sizeof(command));
        if (bytes_read == 1 && command[0] == 0xff){
            std::cout << "INFO: received data request flag (0xff)" << std::endl;
            data_requested = !data_requested;
        }

        // write data to serial port if commanded
        if (!data_requested){
            std::cout << "INFO: IMU is waiting for a data request to start writing data to the port" << std::endl;
        } else{
            ssize_t bytesWritten = _writeData(reinterpret_cast<const uint8_t*>(data), sizeof(data));
            if (bytesWritten == -1) {
                perror("ERROR: failed to write to the port!");
            } else if (bytesWritten != sizeof(data)) {
                std::cerr << "WARNING: Partial write to the port" << std::endl;
            } else {
                std::cout << std::hex << std::setw(4) << std::setfill('0') <<
                "INFO: Wrote " << bytesWritten << " bytes to the port: "
                << "AccelX: 0x" << data[0] << ", "
                << "AccelY: 0x" << data[1] << ", "
                << "AccelZ: 0x" << data[2] << ", "
                << "Checksum: 0x" << data[3] << std::endl << std::dec;
            }
        }

        // simulate the given output_rate
        std::this_thread::sleep_for(std::chrono::microseconds(us_period));

        // kill the loop if we set a run duration
        if (duration_s >=0){
            auto curr_time = std::chrono::steady_clock::now();
            auto curr_runtime = std::chrono::duration_cast<std::chrono::seconds>(curr_time - start);
            loop_condition = (curr_runtime >= duration);
        }
    }
}

bool DummyIMU::_configureSerialPort(){
    int result = tcgetattr(_serial_port, &_options);
    if (result < 0){
        perror("ERROR: failed to tcgetattr() while configuring serial port!");
        return false;
    }

    // Set io baud rate
    result = cfsetispeed(&_options, B9600);
    if (result < 0){
        perror("ERROR: failed to cfsetispeed() while configuring serial port!");
        return false;
    }
    result = cfsetospeed(&_options, B9600);
    if (result < 0){
        perror("ERROR: failed to cfsetospeed() while configuring serial port!");
        return false;
    }

    // NOTE: According to Oracle documentation, "None of the bits in the 
    // c_cflag word have any effect on the pseudo-terminal, except that 
    // if the baud rate is set to B0,". Also, "There is no notion of parity on a 
    // pseudo-terminal, so none of the flags in the c_iflag word that control the 
    // processing of parity errors have any effect. Similarly, there is no 
    // notion of a break, so none of the flags that control the processing of breaks, 
    // and none of the ioctls that generate breaks, have any effect."
    // 
    // Despite this, this method will "pretend" we are interfacing with
    // a real terminal, so some of these flags will be set even if
    // they have no effect. 

    _options.c_cflag &= ~PARENB; // clear parity bit
    _options.c_cflag &= ~CSTOPB; // use one stop bit
    _options.c_cflag &= ~CSIZE; // clear data size bits
    _options.c_cflag |= CS8; // set 1 byte = 8 bits
    _options.c_cflag &= ~CRTSCTS; // disable hardware flow control (default in termios)
    _options.c_cflag |= CREAD | CLOCAL; // allow data reads, disable modem-specific signal lines

    _options.c_lflag &= ~ICANON; // disable canonical mode (raw data only)
    _options.c_lflag &= ~ECHO; // disable echo
    _options.c_lflag &= ~ECHOE; // disable erasure
    _options.c_lflag &= ~ECHONL; // disable new-line echo
    _options.c_lflag &= ~ISIG; // disable interpretation of INTR, QUIT and SUSP characters (raw data only)
    _options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable s/w flow control (default in termios)
    _options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable special handling of received bytes (raw data only)

    _options.c_oflag &= ~OPOST; // disable special handling of output bytes
    _options.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return then newline

    // port uses O_NDELAY by default, so these parameters are ignored.
    _options.c_cc[VTIME] = 0; 
    _options.c_cc[VMIN] = 0;
    result = tcsetattr(_serial_port, TCSANOW, &_options);
    if (result < 0){
        perror("ERROR: tcsetattr() failed while configuring serial port!");
        return false;
    }
    return true;
}

ssize_t DummyIMU::_writeData(const uint8_t* data, size_t size) {
    return write(_serial_port, data, size);
}

ssize_t DummyIMU::_readData(uint8_t* buffer, size_t size) {
    return read(_serial_port, buffer, size);
}

int main(int argc, char** argv) {
    if (argc != 2){
        std::cout << "Expected two arguments, got " << argc << "." << std::endl;        
    }

    ssize_t duration_s = -1;
    // run in debug if we don't want to infinite loop
    #ifdef DEBUG
        std::cout<<"Running temporary debug sim"<<std::endl;
        duration_s = 5;
    #endif

    char *port = argv[1];
    int output_rate = 400;
    DummyIMU imu(port, output_rate);
    if (!imu.openPort()) {
        return -1;
    }
    
    imu.run(duration_s);

    return 0;
}