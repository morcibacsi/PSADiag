# PSA Diag library

### What is it ?

This is a n Arduino library/sketch to send diagnostic frames to PSA cars.

### Why ?

In my previous PSA related projects I needed to set up some ECUs and it would have been more convenient to do it on the bench without attaching the proper BSI (AEE2004, AEE2010). Also would have been nice to provide some diagnostic support as part of my projects so those who don't have access to an AEE2004 BSI because they have an AEE2001 car could easily configure their ECUs.

### But what about the others?

There is at least one another project like this out there which I got the inspiration from:
- https://github.com/ludwig-v/arduino-psa-diag/

Unfortunately it was written in a way that it is impossible to use as a part as another project. So I decided to write my own version which is more modular and can be easily integrated into other projects.

### Usage

Beside the library this repository also contains a sketch which can be used with the serial port. But as the library accepts an incoming command this way the command can come from any source (like a bluetooth serial, or a websocket). All you need to do is to implement the **AbsSer** interface and pass it to the **PsaDiag** class.
The CAN bus interface is also abstracted so you can use any CAN library you want. The only requirement is that it should implement the **ICanMessageSender** interface.

### Liability

This is an experimental software which wasn't thorougly tested, and may not work as intended, there is also a possibility that it breaks your ECU you are trying to interact with. So use it at your own risk.

### Building the project

The project is designed around the ESP32, but it should be easily portable to other platforms as well. It can be built from PlatformIO. In theory it can be built with Arduino as well, but in the limited time I have and the vast amount of versions users can have installed I can't support it. Just use PlatformIO, it guarantees that the code is compiled at your computer just like it did on mine.

### Commands

Take a look at the [commands.md](commands.md) file for the available commands.

### Credits

- vlud for the original project
- altech for the original version of the isotp library (https://github.com/altelch/iso-tp)