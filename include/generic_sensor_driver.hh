#ifndef GENERIC_SENSOR_DRIVER_HH
#define GENERIC_SENSOR_DRIVER_HH

#include <termios.h>
#include <unistd.h>
#include <cstdint>
#include <string>

/**
 * @class GenericSensorDriver
 * @brief A base class for interfacing with generic sensors over a serial connection.
 * 
 * This class provides a basic template for communicating with a sensor 
 * over a serial port, using the core functions of opening/closing the port,
 * reading and writing data, and storing the data in a buffer.
 * stopMonitor and startMonitor must be implemented by derived classes --
 * these indicate when and how to read data from the port.
 * Two possible approaches are to use an interrupt-driven monitor or a polling-driven monitor.
 */
class GenericSensorDriver {
    public:
        /**
         * @brief Retrieves the current value of _raw_data.
         * @return A pointer to the raw data buffer.
         */
        virtual const uint8_t* getRawData() const;

        /**
         * @brief Opens the serial port for communication.
         * @return True if the port was successfully opened, false otherwise.
         */
        virtual bool openPort();

        /**
         * @brief Closes the serial port.
         * @return True if the port was successfully closed, false otherwise.
         */
        virtual bool closePort();

        /**
         * @brief Starts monitoring the sensor data.
         * 
         * This is a pure virtual function and must be implemented by derived classes.
         */
        virtual void startMonitor() = 0;

        /**
         * @brief Stops monitoring the sensor data.
         * 
         * This is a pure virtual function and must be implemented by derived classes.
         */
        virtual void stopMonitor() = 0;

    protected:
        std::string _port_name; ///< Name of the serial port.
        ssize_t _serial_port; ///< File descriptor for the serial port.
        struct termios _options; ///< ettings for the serial port.
        int _bytes_per_read; ///< Number of bytes per read operation
        uint8_t* _raw_data; ///< Buffer for storing raw data from the sensor.

        /**
         * @brief Constructor for the GenericSensorDriver class.
         * @param port_name Name of the serial port.
         * @param options Settings for the serial port.
         * @param bytes_per_read Number of bytes per read operation
         */
        GenericSensorDriver(
            const std::string& port_name,
            const termios& options,
            int bytes_per_read
        );

        /**
         * @brief Destructor for the GenericSensorDriver class.
         */
        virtual ~GenericSensorDriver();

        /**
         * @brief Writes a series of bytes to the serial port.
         * @param data Pointer to the data to be written.
         * @param size Size of the data to be written.
         * @return The number of bytes written, or -1 on error.
         */
        virtual ssize_t _writeData(const uint8_t* data, size_t size);

        /**
         * @brief Reads a series of bytes from the serial port.
         * @param buffer Buffer to store the read data.
         * @param size Size of the buffer.
         * @return The number of bytes read, or -1 on error.
         */
        virtual ssize_t _readData(uint8_t* buffer, size_t size);

        /**
         * @brief Configures the serial port.
         *
         * This method uses the termios struct passed in to the constructor to configure
         * the serial port. For simplicity, this method is protected to prevent
         * potentially dangerous external calls (i.e. while the port is open and in-use).
         * Derived classes should implement their own protocol, such as 
         * an "active" flag, to prevent configuration. There could also be a 
         * "force" flag to force a connection restart before reconfiguring. 
         *
         * @return True if the configuration was successful, false otherwise.
         */
        virtual bool _configureSerialPort();
};

inline GenericSensorDriver::GenericSensorDriver(
        const std::string& port_name,
        const termios& options,
        int bytes_per_read
        ):
            _port_name(port_name),
            _serial_port(-1), 
            _options(options),
            _bytes_per_read(bytes_per_read){
    _raw_data = new uint8_t[bytes_per_read];
}

inline GenericSensorDriver::~GenericSensorDriver() {
    delete[] _raw_data;
    closePort();
}

inline const uint8_t* GenericSensorDriver::getRawData() const {
    return _raw_data;
}

inline bool GenericSensorDriver::openPort() {
    _serial_port = open(_port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (_serial_port == -1) {
        perror("ERROR: failed to open port!");
        return false;
    }
    return true;    
}

inline bool GenericSensorDriver::closePort() {
    if (_serial_port == -1){
        perror("ERROR: there is no connected port to close!");
        return false;
    }
    int result = close(_serial_port);
    if (result == -1){
        perror("ERROR: failed to close file descriptor!");
        return false;
    }
    _serial_port = -1;
    std::cout << "Successfully closed port: " << _port_name << std::endl;
    return true;
}

inline ssize_t GenericSensorDriver::_writeData(const uint8_t* data, size_t size) {
    return write(_serial_port, data, size);
}

inline ssize_t GenericSensorDriver::_readData(uint8_t* buffer, size_t size) {
    return read(_serial_port, buffer, size);
}

inline bool GenericSensorDriver::_configureSerialPort(){
    if (_serial_port == -1){
        std::cerr << "ERROR: cannot configure port that has not been opened!";
        return false;
    }
    int result = tcsetattr(_serial_port, TCSANOW, &_options);
    if (result < 0){
        perror("ERROR: tcsetattr() failed while configuring serial port!");
        return false;
    }
    return true;
}

#endif // GENERIC_SENSOR_DRIVER_HH