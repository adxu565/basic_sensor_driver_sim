#include <iostream>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <atomic>
#include <mutex>
#include <iomanip>
#include "../include/imu_packet.hh"
#include "../include/generic_sensor_driver.hh"

/**
 * \class IMUDriver
 * \brief Handles communication with a simulated Inertial Measurement unit
 *
 * This class manages the IMU sensor communication over a serial port, including
 * interpreting the raw incoming bytes, sending commands, and polling the 
 * sensor for data.
 */
class IMUDriver : public GenericSensorDriver {
public:
    /**
     * \brief Constructs an IMUDriver object.
     * \param port Name of the serial port.
     * \param bytes_per_read Number of bytes per read operation
     */
    IMUDriver(const std::string& port, const termios& options, int bytes_per_read, int _poll_rate_hz);

    /**
     * \brief Destructor for IMUDriver.
     *
     * Stops monitoring and frees _raw_data.
     */
    ~IMUDriver() override;

    /**
     * \brief Calculates a checksum for three 16-bit integers.
     * \param a First 16-bit integer.
     * \param b Second 16-bit integer.
     * \param c Third 16-bit integer.
     * \return The calculated checksum.
     */
    static uint16_t calculateChecksum(uint16_t a, uint16_t b, uint16_t c);

    /**
     * \brief Opens the serial port.
     *
     * Upon opening the port, the driver will attempt to put the port in 
     * exclusive mode. It will close the port if it fails change the mode. 
     *
     * \return True if the port was successfully opened and configured, false otherwise.
     */
    bool openPort() override;

    /**
     * \brief Flushes the output buffer then closes the serial port.
     * \return True if the port was successfully closed, false otherwise.
     */
    bool closePort() override;

    /**
     * \brief Starts monitoring the IMU sensor.
     *
     * Flushes input and output buffers, sends an initialization command, 
     * and starts a separate thread to poll the sensor at _poll_rate_hz.
     */
    void startMonitor() override;

    /**
     * \brief Stops monitoring the IMU sensor.
     *
     * Stops the polling thread and sends a command to stop data transmission.
     */
    void stopMonitor() override;

    /**
     * \brief Gets the class member _raw_data.
     * \return Const uint8_t *pointer to _raw_data.
     */
    const uint8_t* getRawData() const override;

    /**
     * \brief Gets the parsed data, derived from _raw_data.
     * \return Constant IMUPacket reference to the parsed data.
     */
    const IMUPacket& getParsedData() const;

private:
    int _poll_rate_hz; ///< Rate (Hz) that the driver will poll the port for data.
    int _period_us; ///< Polling period in microseconds.

    IMUPacket _parsed_data; ///< Parsed IMU data.

    std::atomic<bool> _listening; ///< Flag indicating if the driver is listening/polling data.
    std::thread _poll_thread; ///< Thread for polling the sensor.
    mutable std::mutex _raw_data_mutex; ///< Mutex for accessing _raw_data.
    mutable std::mutex _parsed_data_mutex; ///< Mutex for accessing _parsed_data.

    /**
     * \brief Writes a sequence of bytes to the serial port.
     * \param data Pointer to the data to write.
     * \param size Number of bytes to write.
     * \return Number of bytes written, or -1 on error.
     */
    ssize_t _writeData(const uint8_t* data, size_t size) override;

    /**
     * \brief Reads a sequence of bytes from the serial port and stores it in _raw_data.

     * \return Number of bytes read, or -1 on error.
     */
    ssize_t _readData();

    /**
     * \brief Configures the serial port settings.
     * \return True if configuration was successful, false otherwise.
     */
    bool _configureSerialPort() override;

    /**
     * \brief Sends a command to the IMU sensor.
     *
     * For simplicity the only command that the IMU will recognize is 0xFF,
     * which tells the IMU to stop writing to the port.
     *
     * \param command The command byte to send.
     */
    void _sendCommand(uint8_t command);

    /**
     * \brief Processes the raw data from the IMU sensor.
     *
     * Data from the IMU comes in 8 byte packets, 2 bytes each for x-y-z data, then 2 for the checksum.
     * Recreate the 16-bit values depending on the desired endianness, 
     * calculate and verify the checksum, and populate _parsed_data.
     * This method could be updated to interpret a more complex communication protocol to handle
     * issues such as byte alignment and malformed data. See readme for more detailed discussion.
     *
     * \param parse_as_little_endian Flag indicating if data should be parsed as little-endian, true by default
     */
    void _processData(bool parse_as_little_endian = true);

    /**
     * \brief Polls the IMU sensor for data.
     *
     * Continuously reads and processes data from the sensor at _poll_rate_hz.
     * Instead of polling, a generic sensor driver could be interrupt-driven,
     * where we use select() to block the thread until we detect data on the port.
     *
     */
    void _pollSensor();
};

IMUDriver::IMUDriver(
        const std::string& port_name,
        const termios& options,
        int bytes_per_read,
        int read_rate_hz): 
            GenericSensorDriver(port_name, options, bytes_per_read), 
            _poll_rate_hz(read_rate_hz),
            _period_us(1000000 / read_rate_hz), 
            _parsed_data(), 
            _listening(false) {
    // TODO: potentially add parameter validation
}

IMUDriver::~IMUDriver() {
    stopMonitor();
    // base destructor will free _raw_data and make sure the port is closed.
}

uint16_t IMUDriver::calculateChecksum(uint16_t a, uint16_t b, uint16_t c){
    uint32_t sum = a + b + c;
    sum = (sum >> 16) + (sum & 0xFFFF);
    sum += sum >> 16;
    return static_cast<uint16_t>(~sum);
}

bool IMUDriver::openPort() {
    if (!GenericSensorDriver::openPort()){
        return false;
    }
    // make sure no other app has this port open
    if (ioctl(_serial_port, TIOCEXCL) == -1) {
        perror("ERROR: Failed to set exclusive mode, port is already occupied!");
        closePort();
        return false;
    }
    return GenericSensorDriver::_configureSerialPort();
}

bool IMUDriver::closePort() {
    if (tcflush(_serial_port, TCIFLUSH) == -1){
        std::cerr << "WARNING: failed to flush output buffer on port" << _port_name << "before closing the port." << std::endl;
    }
    if (!GenericSensorDriver::closePort()){
        return false;
    }
    return true;
}

void IMUDriver::startMonitor() {
    if (!_listening){
        _listening = true;
        // flush input and output buffers in case there is any stale data. 
        // this is a very basic protocol, and there could be scenarios
        // where we still want to process or log old data. But, this implementation
        // is useful if you only want to care about the most recent data.
        if (tcflush(_serial_port, TCIOFLUSH) == -1){
            std::cerr << "WARNING: failed to flush input and output buffers on port" << _port_name << "before driver starts to poll. There may be stale data!" << std::endl;
        }
        // tell the IMU to start writing data
        _sendCommand(0xFF);

        _poll_thread = std::thread(&IMUDriver::_pollSensor, this);
    }
}

void IMUDriver::stopMonitor() {
    if (_listening){
        _listening = false;
        if (_poll_thread.joinable()){
            _poll_thread.join();
        }
        // tell the IMU to stop writing data
        _sendCommand(0xFF);
    }
}

const uint8_t* IMUDriver::getRawData() const {
    std::lock_guard<std::mutex> _raw_data_lock(_raw_data_mutex);
    const uint8_t* data = _raw_data;
    return data;
}

const IMUPacket& IMUDriver::getParsedData() const {
    std::lock_guard<std::mutex> _parsed_data_lock(_parsed_data_mutex);
    const IMUPacket& packet = _parsed_data;
    return packet;
}

ssize_t IMUDriver::_writeData(const uint8_t* data, size_t size) {
    return GenericSensorDriver::_writeData(data, size);
}

ssize_t IMUDriver::_readData() {
    std::lock_guard<std::mutex> _raw_data_lock(_raw_data_mutex);
    int bytes_read = GenericSensorDriver::_readData(_raw_data, _bytes_per_read);
    return bytes_read;
}

bool IMUDriver::_configureSerialPort(){
    return GenericSensorDriver::_configureSerialPort();
}

void IMUDriver::_sendCommand(uint8_t command){
    uint8_t command_packets[1] = {command};
    ssize_t bytesWritten = _writeData(reinterpret_cast<const uint8_t*>(command_packets), sizeof(command_packets));
    if (bytesWritten <=0) {
        perror("ERROR: failed to write command packets to the port!");
    } else {
        std::cout << "INFO: Wrote " << bytesWritten << " bytes to the port: "
        << "Command: 0x" << std::hex << static_cast<int>(command) << std::endl << std::dec;
    }
}

void IMUDriver::_processData(bool parse_as_little_endian){
    uint16_t accel_x; 
    uint16_t accel_y;
    uint16_t accel_z; 
    uint16_t checksum;
    std::unique_lock<std::mutex> _raw_data_lock(_raw_data_mutex);

    if (parse_as_little_endian == true){
        accel_x = (_raw_data[1] << 8) + _raw_data[0];
        accel_y = (_raw_data[3] << 8) + _raw_data[2];
        accel_z = (_raw_data[5] << 8) + _raw_data[4]; 
        checksum = (_raw_data[7] << 8) + _raw_data[6];
    } else{
        accel_x = (_raw_data[0] << 8) + _raw_data[1];
        accel_y = (_raw_data[2] << 8) + _raw_data[3];
        accel_z = (_raw_data[4] << 8) + _raw_data[5];
        checksum = (_raw_data[6] << 8) + _raw_data[7];
    }
    _raw_data_lock.unlock();

    // debug/visualization prints
    std::cout << std::hex << std::setw(4) << std::setfill('0') 
    << "INFO: Received IMU Data: "
    << "AccelX: 0x" << accel_x << ", "
    << "AccelY: 0x" << accel_y << ", "
    << "AccelZ: 0x" << accel_z << ", "
    << "Checksum: 0x" << checksum << std::endl << std::dec;

    uint16_t calculated_checksum = calculateChecksum(accel_x, accel_y, accel_z);
    if (calculated_checksum == checksum) {
        std::lock_guard<std::mutex> _parsed_data_lock(_parsed_data_mutex);
        // update data only if its valid
        _parsed_data.accel_x = accel_x;
        _parsed_data.accel_y = accel_y;
        _parsed_data.accel_z = accel_z;
        _parsed_data.checksum = checksum;
        std::cout << "INFO: checksum verified" << std::endl;
    } else{
        std::cerr << "ERROR: checksum of received packet (" << checksum << ") does not match calculated checksum (" 
        << calculated_checksum << ")! Throwing out packet." << std::endl;
    }
}

void IMUDriver::_pollSensor() {
    ssize_t bytes_read;
    while(_listening){
        // tasks to do each time we poll
        bytes_read = _readData(); // get raw bytes
        if (bytes_read == _bytes_per_read) {
            // Successfully read 8 bytes, parse data however as desired
            // from the raw stream.
            // For now we assume the sensor sends only perfect n-byte packets,
            // and we just parse each packet directly. See readme for discussion
            _processData();
        } else if (bytes_read > 0) {
            std::cerr << "WARNING: Incomplete data received: " << bytes_read << " bytes" << std::endl;
        } else if (bytes_read == -1){
            std::cerr << "ERROR: Unable to read from port: " << _port_name << std::endl;
        } else {}

        // simulate the poll rate
        std::this_thread::sleep_for(std::chrono::microseconds(_period_us));
    }
}

// helper function to configure default termios options
bool setDefaultOptions(termios options){
    // Set io baud rate
    int result = cfsetispeed(&options, B9600);
    if (result < 0){
        perror("ERROR: failed to cfsetispeed() while configuring serial port!");
        return false;
    }
    result = cfsetospeed(&options, B9600);
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

    options.c_cflag &= ~PARENB; // clear parity bit
    options.c_cflag &= ~CSTOPB; // use one stop bit
    options.c_cflag &= ~CSIZE; // clear data size bits
    options.c_cflag |= CS8; // set 1 byte = 8 bits
    options.c_cflag &= ~CRTSCTS; // disable hardware flow control (default in termios)
    options.c_cflag |= CREAD | CLOCAL; // allow data reads, disable modem-specific signal lines

    options.c_lflag &= ~ICANON; // disable canonical mode (raw data only)
    options.c_lflag &= ~ECHO; // disable echo
    options.c_lflag &= ~ECHOE; // disable erasure
    options.c_lflag &= ~ECHONL; // disable new-line echo
    options.c_lflag &= ~ISIG; // disable interpretation of INTR, QUIT and SUSP characters (raw data only)
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // disable s/w flow control (default in termios)
    options.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // disable special handling of received bytes (raw data only)

    options.c_oflag &= ~OPOST; // disable special handling of output bytes
    options.c_oflag &= ~ONLCR; // prevent conversion of newline to carriage return then newline

    // port uses O_NDELAY by default, so these parameters are ignored.
    options.c_cc[VTIME] = 0; 
    options.c_cc[VMIN] = 0;

    return true;
}

int main(int argc, char** argv) {
    if (argc != 2){
        std::cout << "Expected two arguments, got " << argc << "." << std::endl;
        return -1;        
    }

    #ifdef DEBUG
        std::cout<<"Running temporary debug driver"<<std::endl;
    #endif
    
    char *port = argv[1];
    int read_rate_hz = 800; // read at double the freq of the sensor's update rate
    int bytes_per_read = 8;
    termios options;
    setDefaultOptions(options);

    IMUDriver imu_driver(port, options, bytes_per_read, read_rate_hz);

    if (!imu_driver.openPort()) {
        return -1;
    }

    imu_driver.startMonitor();

    // run in debug if we don't want to infinite loop
    #ifdef DEBUG
        sleep(5);
        imu_driver.stopMonitor();
        return 0;
    #endif 

    while(true){}
    imu_driver.stopMonitor();
    return 0;
}
    
