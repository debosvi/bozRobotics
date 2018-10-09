# Time stamp of the frame as received by the low level driver
# Can be left empty when sending frames.
time    stamp

# ID of the received frame or of the frame to send
uint32  id

# Data bytes in the wire order (byte 0 is the first that has been
# received or the first that will be sent). 
# The DLC is the length of the array.
uint8[] data	