The evt21Streamer module is responsible for owning the driver object from genx320Drvier and piping the evt21 bytes from the V4L2 buffer to another program. 

Another program will own the evt21Streamer object and utilize it to copy EVT21 bytes and perform decoding and other operations on the data.
