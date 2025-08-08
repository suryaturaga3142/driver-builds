# driver-builds

Arduino may not be too powerful, but it has some really good libraries that can make beginners into skilled creators. STM32 is far more powerful, and has access to many tools and more peripherals in general. I have quite a few sensors and actuators for different uses. Instead of copying over and pasting huge blocks of C code everytime, it's easier to just have a bunch of libraries that I can import!

I'm building a small collection of some common actuators, inspired by the Arduino style and built for the STM32 workflow! For reference, I'm currently using a NUCLEO-F767. The repo is a pretty big work in progress, since there's always more functionality to add. Everything can operate in regular mode, use interrupts and DMA, interact with watchdogs, etc. Idt this repo can ever be considered "complete" lol.
