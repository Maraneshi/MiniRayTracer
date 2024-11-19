# MiniRayTracer
A CPU path tracer, runs on Windows and Linux.  
Initial implementation and scenes based on [Peter Shirley's minibook series on ray tracing](https://in1weekend.blogspot.com/).

### Help

Get command line options with *--help*. Default parameters are currently in *cmdline_parser.h*.

### Dependencies
* C++20 compatible Clang or Visual Studio 2022
* Linux only: SDL2

### Example output images:
![500x500, 1764 samples per pixel, 3 threads, 32 bounces](https://i.imgur.com/2VsYmUf.png)
![450x400, 10000 samples per pixel, 4 threads, 32 bounces](https://i.imgur.com/wYSJHR0.png)

Some more process gifs [here](https://imgur.com/a/lBOdJ).