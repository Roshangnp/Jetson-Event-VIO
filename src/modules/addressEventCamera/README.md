This module will utilize the EVT21 modules to generate and maintain event histograms, and event time surfaces. It will give parent programs access to the data for further processing. For example the optic flow module may utilize this module
by copying the histogram data into another location in memory to compute optic flow.

This module also includes a light weight EVT21 decoder.

Event histogram buffer structure:

The histograms class manages the histogram buffers and be default owns 5 histogram buffers (each being about 200Mb). 

Each histogram gives the sum of all events per pixel over a time interval where positive events are +1 and negative events are -1. The time interval are defined by quantity (size_t) of TIME_HIGH evt21 words, default of 40 which represents about 2.5ms. Defining the interval this way makes updating and maintaining histograms more efficent, as event words between TIME_HIGH words are not garanteed to arrive in chronological order, and otherwise would need to be handled.

The histogram_buffer class carries the histogram_array data and provides an functions to update the histogram with evt21 words and reset the data. 
