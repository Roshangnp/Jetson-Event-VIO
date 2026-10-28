# Jetson-Event-VIO
Visual inertial odometry based on the prophese genx320 address event camera made for the jetson orin nx.

Uses events to generate velocity estimates using ego motion and then sends it to a connected PX4 flight controller.

## Current goal

The optic flow pipeline being worked towards:

```
histogram_pair -> filter -> spatial and temporal derivative convolution (valid pixels) -> Lucas-Kanade (valid pixels)
```

- **filter** is one of:
  - second-order Gaussian convolution with the Hessian ridge criterion
  - minimum magnitude threshold
  - nothing (the next step processes every pixel)
- **spatial derivative convolution** is either Savitzky-Golay (SGF) or first-order Gaussian convolution.
- **temporal derivative convolution** runs on both histograms of the pair; their difference is the second-order temporal derivative used by LK.
- Each step can be a CUDA kernel or a CPU process.
