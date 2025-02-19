# Basic-Serial-Port

Is a lightweight and portable C and C++ library 
providing a simple API for serial port communication on Linux. 

Designed specifically for robotics and embedded systems, 
it prioritizes a small footprint and avoids extra dependencies 
to streamline the installation process.  
Leveraging versatile I/O and `ioctl()` for low-level control, 
it offers a straightforward way to read and write data using 
familiar standard library functions.  

This library is intended for users familiar with their 
hardware configurations, and therefore does not include 
features such as automatic port discovery or baud rate detection. 
 
This library is intended for communication with non-canonical devices.

---

## Table of Contents

- [Features](#features)
- [Building](#building)
- [Installation](#installation)
- [Examples](#examples)
- [License](#license)
- [Contact](#contact)

---

## Features

*   **Portable:**  Written in standard C and C++ and designed for Linux systems.
*   **No External Dependencies:**  Easy to integrate into embedded systems and projects with minimal overhead.
*   **Configurable:** Supports setting baud rate, parity, stop bits and flow control.
*   **Deterministic:** Supports timeouts or minimum number of bytes to block read functions.
*   **Documentation:** Documentation done in doxygen.
*   **Tested:** Includes a suite of unit tests on real hardware.

---

## Building

Clone the repository (if not already done):

```bash
$ git clone https://github.com/FDanielPacheco/LibLinux-Serial_C.git

$ cd Basic-Serial-Port
```

If it's only necessary to install the library jump to section install, otherwise to build the library:

```bash
$ make build
```

In case it's necessary to remove the build artifacts and build again:

```bash
$ make clean

$ make build
```
---

## Installing

After cloning the repository, inside the LibLinux-Serial_C/, the command will install the library on the system, to be able to include the library in any project:

```bash
$ make install
```
---

## License

This project is licensed under the [GNU_V3 License](./../../LICENSE).

---

## Contact

For any questions or issues, please reach out via email at pacheco.castro.fabio@gmail.com or fabio.d.pacheco@inesctec.pt.