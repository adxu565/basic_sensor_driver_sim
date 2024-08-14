PACKAGES: 

The only package to install is xterm, which is used for visualizing the
IMU sensor sim in a new terminal window. g++ 13.2.0 was used to compile the project.
-------------------------------------------------------------------------------
sudo apt-get install xterm
-------------------------------------------------------------------------------

USAGE:

Use make_debug for a temporary simulation, otherwise the driver/sensor will
run in an infnite loop (use ctrl-c to exit the script).
-------------------------------------------------------------------------------
make
./run_drivers_and_sensors.sh
-------------------------------------------------------------------------------

IMPLEMENTATION DISCUSSION:

I saw several approaches to this. One was to implement a poll-based driver,
the other was to implement an interrupt based driver. I decided to go with
a poll-based driver because I simulated an IMU -- from my work experience, 
I've seen space vehicles favor polling to avoid timing issues and have the safety 
of a predictable, pre-defined bus schedule when there are many tasks to fit within a given run-step.
If the sensor was something like a laser trip wire for a security system, an interrupt-based
driver may be more appropriate for reduced latency and CPU efficiency. 

From here, I then needed to decide if I wanted the driver to run in a parallel
thread, constantly reading the port at some desired rate, or just have it 
run in the main thread. From reading documentation online, I've seen some sources
(such as the ArduPilot library) run the driver in the main thread since UART
is buffered and and collects data in the background.
I decided to make my implementation run in a parallel thread, so that the 
application using the driver could subscribe to data at it desires while
the driver stays as up-to-date as possible with the incoming data.

For the communication across the serial port, I assumed an ideal scenario 
where we have fixed packet sizes, and always read/write the number of expected bytes.
This way I could simply assume that whatever 8 bytes the driver reads directly
corresponds to 1 packet, and there is no stale data. While I flush the buffers
to ensure there's no stale data at the start/stop, this assumption wouldn't
work if incoming data started to fill the buffer during runtime. Another edge case
would be if we have partial packet data, but want to continue parsing until 
we have a potentially valid packet. Then we would need a protocol for message
alignment. One possible way is to have the checksum/initial 2 bytes at the start
of the packet. We can get an 8-byte sequence, verify the checksum, toss the first byte if invalid, 
and add the next byte to the 8-byte sequence.

A few other things that could be added:
- a generic sensor class for the IMU
- a more realistic IMU simulation
- doxygen comments for the DummyIMU
- better logging structure
- use output/debug logs to create unit tests and verify communications
- child classes of GenericSensorDriver, GenericPollingDriver and GenericInterruptDriver.

