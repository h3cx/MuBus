<p align="center">
  <img src="https://raw.githubusercontent.com/h3cx/MuBus/main/images/mubus.png"
       width="30%" />
</p>
A high performance, low overhead serial communication protocol

> [!IMPORTANT]
> μBus currently only support Arduino and other related boards that have a `HardwareSerial` object

# Installation
μBus is currently aimed at being used with PlatformIO, it is available on the PlatformIO registry but likely not up to date due to active development.
In order to install μBus run the following in your PlatformIO project root folder:
```
pio pkg install -l "https://github.com/h3cx/MuBus.git"
```
> [!IMPORTANT]
> Expect regular breaking changes during initial development.

# Usage
## Constructor
The basic constructor follows the following pattern:
```cpp
MuBusNode(HardwareSerial& port, uint8_t addr)
```
For example, on an Arduino Mega using `Serial1` and with a device address of `0x01` this would give:
```cpp
MuBusNode node(&Serial1, 0x01);
```
You will still be required to call `Serial1.begin()` seperately as μBus doesn't handle this yet

## Sending Data
Once you have set up your node, you can now send data from it. Currently the only supported mode is broadcast (`dest_addr = 0x00`).
To broadcast you use the following function:
```cpp
bool broadcast(uint8_t *buf, uint16_t len)
```
The maximum `len` is 506 bytes, this is a limitation of μBus which has a maximum dataframe of 512 bytes.
An example usage sending a custom data struct:
```cpp
data_struct_t some_data;
node.broadcast(&some_data, sizeof(some_data));
```

## Receiving Data
In order to receive data, the parse function can be used:
```cpp
bool parse()
```
This function will search for header bytes in the serial interface, and only once it finds a matching packet will it fully read it. If it reads a valid data frame, it will return true. This function will also filter the destination address and check it against its address that was set in the constructor, it will only decode packets that are destined to it or that are broadcast (0x00).
This function is called in a loop to form the parser:
```cpp
while(true) {
  if (node.parse()) {
    //do something
  }
}
```
In order to access the payload the following functions can be used:
```cpp
uint8_t *getPayload();
uint16_t getPayloadSize();
```
