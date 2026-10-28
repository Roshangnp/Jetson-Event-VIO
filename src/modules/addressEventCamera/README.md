This module will utilize the EVT21 modules to generate and maintain event histograms, and event time surfaces. It will give parent programs access to the data for further processing. For example the optic flow module may utilize this module
by copying the histogram data into another location in memory to compute optic flow.

This module also includes a light weight EVT21 decoder.

EVT21 decoder:

evt21Decoder holds all of the EVT2.1 bit layout constants, so other code never shifts or masks words itself. It provides the word type, timestamps, the decoded TIME_HIGH value and a wrap safe TIME_HIGH duration, an event word's x, y and 32 bit valid mask (bit i set means pixel x + i fired), and is_valid_event(), which rejects corrupted event words (x not aligned on 32, or the pixels falling off the sensor).

Event histogram buffer structure:

The histograms class manages the histogram buffers and by default owns 5 histogram buffers (each being about 200 KB).

Each histogram gives the sum of all events per pixel over a time interval where positive events are +1 and negative events are -1. The time interval are defined by quantity (size_t) of TIME_HIGH evt21 words, default of 40 which represents about 2.5ms. Defining the interval this way makes updating and maintaining histograms more efficent, as event words between TIME_HIGH words are not garanteed to arrive in chronological order, and otherwise would need to be handled.

The histogram_buffer class carries the histogram_array data and provides functions to update the histogram with evt21 words and reset the data. It also stores the window's start and current TIME_HIGH (decoded values), a sequence number, and a state:

- FREE: never filled, or released; contents not needed.
- FILLING: the buffer currently being written by updateHistograms (only one at a time).
- READY: window complete, available to dequeue.
- HELD: handed to the parent program; never written until released.

Only the histograms class changes states. getHistogram() returns the data only when the buffer is READY or HELD.

Filling histograms:

The parent program passes every buffer from evt21Streamer (or evt21Streamer_fromFile) to histograms::updateHistograms(buffer, word_count). One buffer is active at a time. Events before the first TIME_HIGH word are dropped. When the FILLING window reaches the configured width, it becomes READY and the next buffer is picked: a FREE one if there is one, otherwise the oldest READY one (HELD buffers are never picked). It is reset, starts at the same TIME_HIGH and becomes FILLING, so windows have no gaps. Each window gets a sequence number one higher than the last.

Corrupted event words are skipped and counted rather than stopping the stream; getBadWordCount() returns the count.

Reading histograms (pair protocol):

dequeueHistogramPair() returns {newer, older}: the newest ready histogram and the one just before it in time. Both stay protected from being overwritten until released. It returns {nullptr, nullptr} if there is no pair newer than the last one returned, or if the older window has already been overwritten. The parent program never gets the same pair twice, but consecutive pairs overlap by one: {C, B} can follow {B, A}. If the parent program falls behind it gets the newest pair, skipping windows it missed; a buffer still HELD from the previous pair that is not part of the new one is released.

dequeueNextHistogramPair() works the same way but returns pairs in order instead of skipping to the newest: the oldest ready window after the last pair returned, paired with its predecessor. One evt21 buffer can complete several windows, so to consume every window call it in a loop after each updateHistograms() (queueing each pair) until it returns {nullptr, nullptr}. This keeps up as long as one evt21 buffer completes at most 10 windows (HISTOGRAM_BUFFER_COUNT = 12 buffers minus 1 FILLING and 1 HELD); beyond that the oldest ready windows are overwritten and it skips to the oldest pair still available. Use dequeueHistogramPair() when only the latest motion matters (optic flow) and dequeueNextHistogramPair() when every window matters (logging).

When done with a pair, call queueHistogramPair(). This releases only the older buffer (HELD to FREE); the newer one stays HELD because it is the older half of the next pair. A pair must be queued before the next dequeueHistogramPair(), the same as releasing a streamer buffer.

This is not thread safe yet. updateHistograms and the dequeue/queue calls must run on the same thread for now.
