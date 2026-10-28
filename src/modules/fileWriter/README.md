The fileWriter module logs pipeline data to binary files in real time without slowing the pipeline down.

writeHistograms:

initialize(path) creates (or truncates) the file and allocates a ring of 32 record slots once (about 6.5 MB). start() starts a background writer thread. tryWrite(buffer) copies a READY or HELD histogram_buffer into a free slot and returns immediately, so the parent program can queue the histogram pair right away; the writer thread drains the slots to disk. If every slot is full, tryWrite returns false and counts a drop (droppedCount()) instead of waiting on the disk. stop() writes everything still queued, joins the thread, syncs and closes the file.

Only the ring of slots is shared between the threads, so the evt21Streamer -> histograms pipeline stays single threaded. A write error in the writer thread is rethrown by the next tryWrite().

File layout: no file header, just back to back 204,808 byte records in native byte order (little endian on Jetson): uint64_t time_high_start (decoded TIME_HIGH the window began at) followed by the 320x320 int16_t histogram, row major. In Python:

    np.fromfile(path, dtype=[('t', '<u8'), ('hist', '<i2', (320, 320))])

Consecutive pairs from dequeueHistogramPair() overlap, so write only the newer histogram of each pair (and the older one of the first pair) to log every window once.

writeFlows:

Identical to writeHistograms, but logs optic flow: tryWrite(flow, time_high) copies a flow_field and optic_flow::getTimeHigh() into a free slot. The ring has 8 slots (about 9.2 MB), not 32, because one flow_field is about 1.15 MB at a dense 11x11 grid. Logging every window would be about 450 MB/s (390 windows/s x 1.15 MB), so log only some windows.

File layout: no file header, back to back flow_records in native byte order, written straight from memory, so the compiler's padding is included (static_asserts in writeFlows.h fail the build if the layout changes). One record (24 + GRID_HEIGHT x GRID_WIDTH x 12 bytes: 1,153,224 at a dense 11x11 grid):

    offset  0  uint64  time_high        optic_flow::getTimeHigh() (newer window start + 40)
    offset  8  uint64  time_high_start  flow_field: older window start
    offset 16  uint64  time_high_end    flow_field: newer window end (equals time_high)
    offset 24  GRID_HEIGHT x GRID_WIDTH flow_patches, row major, 12 bytes each:
               0  float[2]  flow (px/s, x right, y down)
               8  bool      valid
               9  3 bytes padding

In Python (G = GRID_WIDTH = GRID_HEIGHT: 310 for PATCH_WIDTH 11, 318 for 3, at GRID_STRIDE 1; grid point (gy, gx) is centred on pixel
(gy + PATCH_RADIUS, gx + PATCH_RADIUS) at stride 1):

    patch = np.dtype({'names': ['flow', 'valid'], 'formats': [('<f4', 2), '?'],
                      'offsets': [0, 8], 'itemsize': 12})
    record = np.dtype({'names': ['t', 't_start', 't_end', 'patches'],
                       'formats': ['<u8', '<u8', '<u8', (patch, (G, G))],
                       'offsets': [0, 8, 16, 24], 'itemsize': 24 + G * G * 12})
    flows = np.fromfile(path, dtype=record)
