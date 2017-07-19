High speed ADC example

Example description
This examples shows how to setup and use the high speed ADC (HSADC) that
is on some LPC43xx devices. The example shows hoe to configure the HSADC
for varying clock rates, setup the HSADC, and perform DMA based HSADC
sampling. The examples also shows how the use threshold detection for
determing when the analog signal goes outside the threshold limits.

The HSADC peripheral is directly clocked from the CGU/CCU and doesn't
include a divider before the peripheral block. Because of this, setting
up the HSADC sample clock is more complex than other peripherals. This
example provides a detailed function for setting up the ADC clock and
power setup at various clock rates.

Although the example converts all the 3 available channel inputs, only
data from channel 0 will be printed on UART.

Special connection requirements
On LPC Link2 Board Analog input should be provided on Analog header [J4]
PIN-4. The voltage range must be between 0.05v to 1v. Connect PIN-8 of
Serial expansion header [J3] which is UART_TX and ground to an FTDI cable's
RX and Ground respectively, to see the converted digital value in an UART.

IMPORTANT: The analog input pin is not 5V tolorant hence care must be
taken not to feed any input voltage > 3V to this pin.







