# Experimental Data

This directory is reserved for representative experimental recordings associated with the six validation scenarios:

- rectangles: 30 × 40 cm, 50 × 70 cm, and 70 × 50 cm;
- equilateral triangles: 30 cm and 50 cm sides;
- circle: 40 cm diameter.

Each CSV recording should identify the test, acquisition date, software revision, controller parameters, and sampling frequency. The validated acquisition and control frequency is **50 Hz**.

The recordings may include chassis commands, wheel-speed references and measurements, PWM commands, cumulative encoder ticks, and the estimated pose `(x, y, theta)`.

Odometric closure values are internal consistency indicators and must not be interpreted as absolute positioning errors in the absence of an independent external localization reference.
