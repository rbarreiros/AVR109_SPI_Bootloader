
SPI Bootloader implementing AVR109 protocol

This bootloader was intended for the Raspberry Pi SPI to DMX Modules
where it can upgrade the module through SPI.

The bootloader on system start waits X seconds for a enter programming
code on the SPI bus. After than, it answers to the regular AVR109 
protocol (except some functionality not implemented/needed). When
leaving programming, cleans up and starts application.

Rui Barreiros <rbarreiros@gmail.com>
