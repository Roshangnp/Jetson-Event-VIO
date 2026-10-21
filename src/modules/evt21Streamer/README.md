The evt21Streamer module is responsible for owning the driver object from genx320Drvier and piping the evt21 bytes from the V4L2 buffer to a system memory buffer. 
The evt21Streamer object will give any other program concurrent access to the evt21 stream.
For example, a program may need to access the stream to decode the EVT21 and maintain a real time event histogram
