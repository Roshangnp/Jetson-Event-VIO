The fileWriter module logs pipeline data to binary files in real time without slowing the pipeline down.

writeHistograms:

initialize(path) creates (or truncates) the file and allocates a ring of 32 record slots once (about 6.5 MB). start() starts a background writer thread. tryWrite(buffer) copies a READY or HELD histogram_buffer into a free slot and returns immediately, so the parent program can queue the histogram pair right away; the writer thread drains the slots to disk. If every slot is full, tryWrite returns false and counts a drop (droppedCount()) instead of waiting on the disk. stop() writes everything still queued, joins the thread, syncs and closes the file.

Only the ring of slots is shared between the threads, so the evt21Streamer -> histograms pipeline stays single threaded. A write error in the writer thread is rethrown by the next tryWrite().

File layout: no file header, just back to back 204,808 byte records in native byte order (little endian on Jetson): uint64_t time_high_start (decoded TIME_HIGH the window began at) followed by the 320x320 int16_t histogram, row major. In Python:

    np.fromfile(path, dtype=[('t', '<u8'), ('hist', '<i2', (320, 320))])

Consecutive pairs from dequeueHistogramPair() overlap, so write only the newer histogram of each pair (and the older one of the first pair) to log every window once.
