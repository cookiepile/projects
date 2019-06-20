# 1 Overview
The goal of this project was to implement a peer-to-peer communication system, supporting a carrier sense
multiple access (CSMA) protocol.
The construction phase required building a node capable of interfacing with a PC so that text
commands can be entered by the user. Based on the commands, subsequent transmission on the 2-wire
RS-485 bus to other nodes occurs.
The node also extracted information out of the asynchronous data stream and to control three or more
devices. It sent acknowledgements back to the controller over the RS-485 physical layer to indicate
successful receipt of the data. The transceiver also sent unsolicited messages back to the controller
in response to changes in input device status.

# 2 Two-Wire RS-485 Link
The RS-485 interface allows up to 32 standard loads (devices nodes) on a shared data bus.
In the RS-485 2-wire configuration, a single differential pair (link) was used. The controller and the
device nodes shared the same link for communications. The data rate was 38400 baud. A termination
of 120 ohms was added across to the physical ends of a long wire run to allow the bus to be
properly terminated.
Since the pair can be driven by more than one node at a time, a potential existed for a collision on the bus.
In this project, any message sent had to be repeated at a delay calculated by a binomial exponential backoff
algorithm if an acknowledgement was not received in an acceptable period of time. To prevent the
possibility of duplicate messages being received, a message identifier wass attached to each transmitted
message to allow the recipient to determine message uniqueness.
A data was transmitted in groups of 11 bits, consisting of a start bit (‘0’), a byte, an address (‘1’) / data bit
(‘0’), and a stop bit (‘1’).

# 3 Node Hardware
Microcontroller:
An ARM M4F core (TM4C123GH6PMI microcontroller) 
Serial interface:
If using the EK-TM4C123GXL evaluation board, then the UART0 tx/rx pair is routed to the ICDI that
provides a virtual COM port through a USB endpoint.
RS-485 interface:
A SN75HVD12 is used to convert the CMOS logic level signals to/from the PIC to the differential signals
used for the 2-wire RS-485 interface. The receiver enable (RE) pin of the SN75HVD12 is always
enabled. Since only one node can talk on the bus at a time, the ARM asserts the driver enable (DE) pin
of the SN75HVD12 only when it needs to transmit on the bus.
Power LED:
Yellow LED connected with a 470ohm current limiting resistor to indicate power on the interface board.
TX LED:
Red LED connected with a 470ohm current limiting resistor to a GPO on the controller. 
RX LED:
Green LED connected with a 470ohm current limiting resistor to a GPO on the controller.
Output device:
3 devices such as a light bulb (low voltage only), servo, RGB LED, or
monochrome LED (not red, green, or yellow as those are used above)
Connections:
A 2-pin terminal block used to provide access to the RS-485 signals. 

# 4 Suggested Parts List
Part Quantity
EK-TM4C123GXL (rs-232 receiver/driver) 1
SN75HVD12 (rs-485 transceiver) 1
Yellow LED (power) 1
Red LED (tx) 1
Green LED (rx) 1
470ohm resistor (led current limiters) 3
0.1uF capacitor (bypassing transceiver) 1
8pin 300mil socket (sn75hvd12) 1
10x2 100mil pitch unshrouded header 2
2-pin terminal block 1
Wire (22-24 AWG solid wire, 3 colors) 1
Power supply of your choosing if not using the
EK-TM4C123GXL
1
PC board 1
Device under control and interfacing Varies
Solder, iron, needle-nose pliers, diagonal
cutters, safety glasses, wire, cable, …
1 each

# 5 Data Packets
A packet consisted of the following bytes in order:
DST_ADDRESS: the address of the node that will receive the packet (with the 9th bit = ‘1’)
SRC_ADDRESS: the address of the node that sent the packet
SEQ_ID: a unique packet sequence number (can be a modulo 256 value)
COMMAND: the action being reported (the “verb”)
CHANNEL: the channel of the node to which this message applies (the “noun”)
SIZE: the number of data bytes being included
DATA[n]: n = 0, 1, … SIZE-1 bytes of data specific the packet being sent
CHECKSUM: the 1’s compliment of the modulo-256 sum of all 6+SIZE bytes in the packet
Addresses 0-254 are designed for unicast addressing of nodes. Address 255 is a broadcast address that
instructs all nodes to process the packet. Address 0 is reserved for the controller.

# 5 Node Software
User Interface:
The controller shall provide a text-based user interface that can be accessed with a terminal emulator
program running on the PC. The data format is 115200baud, 8N1. At a minimum, the following
command must be supported:
All commands below should be case-insensitive.
On startup, the node should send “Ready” to the host machine on the serial interface.
The PC can send several commands to the controller:
If “reset A” is received from the PC, a reset is sent to address A.
If “cs ON” or “cs OFF” is received from the PC, carrier sense detection is enabled or disabled.
If “random ON” or “random OFF” is received from the PC, then random retransmissions is enabled or
disabled.
If “set A, C, V” is received from the PC, where A is the address, C is the channel, and V is the value, a set
command shall be sent.
If “get A, C” is received from the PC, where A is the address and C is the channel, a data request shall be
sent.
If “poll” is received from the PC, a poll request shall be sent.
If “sa A, Anew” is received from the PC, where A is the current address and Anew is the new address, the
controller shall send a set address command.
If “ack on” is received from the PC, the controller shall request an acknowledgement for each command
sent.
If “ack off” is received from the PC, the controller shall not request an acknowledgement for any command
sent.
If an errant command is received from the PC, an “Error” response shall be sent.
After a valid command is received from the PC, the controller shall respond with “Queuing Msg N”, where
N is the unique message ID assigned to the message.
Any time a message is transmitted on the RS-485 bus, a node shall send a message “Transmitting Msg
N, Attempt M” to the PC showing the message number (N) and the transmission attempt (M). If
acknowledgement was requested and the maximum number of retransmissions was sent without
acknowledge, an “Error Sending Msg N” message shall be sent to the PC.
Any poll responses and data reports received on the RS-485 bus shall be sent to the PC in a text format
of your choosing.
Packet Transmission:
If carrier sense is enabled, then the node should determine that a carrier is not active prior to starting
transmission. While this does not ensure that a subsequent transmission will be contention-free, it helps
to mitigate collisions.
To address a node, the packet starts with an 8-bit node address with a 9th bit set to ‘1’. Subsequent data
transmissions have the 9th bit cleared. For each new message (not retransmissions, the SEQ_ID field
should be incremented, modulo 256.
When a message is sent, a request can be made to require an acknowledgement or response. If the
sending node does not receive an acknowledgement within a period of time (T×2n; where n = 0 initially),
then it shall;
1. Increment n.
2. Wait a fixed time T0.
3. If random retransmission is enabled, wait a random period of time over an interval of 0 to T×2n
seconds. If random retransmission is disabled, instead wait a period T×2n seconds (debug
mode).
4. Reattempt transmission using the identical message contents (same SEQ_ID).
This procedure is repeated for subsequent timeouts for 0 < n ≤ N; where N is the maximum number of
retries allowed.
Packet Reception:
If an address match is detected, then the receiver shall collect all data bytes in the packet. Based on the
contents of the packet, an acknowledgement may be sent. If the message received is an
acknowledgement of a message earlier transmitted, then cross-check the received SEQ_ID with pending
acknowledgements so that retransmissions stop.
Upon receiving a message requesting an acknowledgement or response or an input device changing
state, the peripheral shall send a packet. If the receiving node does not receive an acknowledgement
within a period of time (T×2n; where n = 0 initially), then it shall:
1. Increment n.
2. Wait a fixed time T0.
3. If random retransmission is enabled, wait a random period of time over an interval of 0 to T×2n
seconds. If random retransmission is disabled, instead wait a period T×2n seconds (debug
mode).
4. Reattempt transmission using the identical message contents (same SEQ_ID).
This procedure is repeated for subsequent timeouts for 0 < n ≤ N; where N is the maximum number of
retries allowed.
TX LED:
The TX LED shall blink to indicate that a message has been transmitted. If the number of retries is
reached, then the TX LED shall be illuminated to indicate an error (the next transmission attempt will blink
and then clear this “error” condition on the LED).
RX LED:
The RX LED shall blink to indicate that a message addressing this node (broadcast or unicast) has been
received by the node. This LED shall blink on power-up to indicate that the controller has booted
correctly.
Output Device(s):
The software shall control one or more devices, each assigned to a channel.
Input Device(s):
Each input device should be assigned a channel. The software shall send a Value message on the
reverse link when the input device status changes or when a Get command is received.
Control Commands:
When a control command is received, the output device selected shall be controlled. Some devices, such
as RGB triplets, LCD displays, and serial devices can ignore these commands.
Sensor Commands:
When a Sensor Request command is received, the input device should be read and the value should be
returned in a Sensor Data command.
Set Address Command:
If a Set Address command is received, then the address of the node shall be changed. The new address
should be written to non-volatile memory so that the value is persistent after power loss.
Poll Request Command:
If a Poll Request command is received, the node shall transmit its address in a Poll Response message.
Reset Command:
If a reset command is received, the node shall reboot.
