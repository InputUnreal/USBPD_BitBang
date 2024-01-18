# USBPD_BitBang
BitBang of the USB Power Delivery Protocol For STM32

Use at own risk, program was writen as a demonstration that the USB Power Delivery Protocol 
could be BitBanged on an STM32F0 (ARM Cortex M0) running and 48MHZ internal oscillator.

Program Uses TIM3 in capture and compare mode to "read" data from CC1/CC2 
and uses DMA write to the I/O port on timer overflow to send data.

The program is the VERY BASIC only sends GoodCRC and request the last object it recieves from the source.
NO ERROR HANDLING or CRC checks. 

The signaling for USBPD requires something like 33 Ohm Impedance and 0 - 1.ish volts.
This is achived with diodes to reduce the 3.3 to 1.ish volts and a NPN transitor. (example circuit provided)

Program could use ALOT of work however, I needed it to work for only a couple of minutes.

A logic Anaylzer like a Saleae Logic 8 / Saleae Clone is highly recommended is you want to use this program.  
