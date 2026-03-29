ESP32 FFT Spectrum Analyzer
A real-time wireless spectrum analyzer built on two ESP32 microcontrollers. 
The system samples an analog input signal, computes a 512-point FFT using the ESP-DSP library, transfers the result over SPI, and displays a live spectrum on a browser-based interface served over WiFi — with no PC or external software required.

Features:
1) 512-point FFT using ESP-DSP
2) 80 kHz sample rate → 40 kHz Nyquist, 156 Hz/bin resolution
3) Real-time SPI transfer between two ESP32s (2 MHz, 1060-byte packets)
4) WiFi Access Point on ESP32 #2 — no router needed
5) Browser-based spectrum display (pure Canvas 2D, zero dependencies, no internet required)
6) Live metadata: peak frequency, peak magnitude, ADC min/max/mean, sample rate, frame count
7) 3-task FreeRTOS architecture — ADC sampling never blocked by FFT or SPI
8) Ping-pong buffer design — zero dropped samples during FFT computation
9) Magic-word packet validation on SPI slave side
10) Hann windowing for reduced spectral leakage
11) DC offset removal before FFT

Data Flow:
Analog signal → ADC (80kHz) → ping-pong buffer → 512pt FFT → SPI packet → WiFi → Browser

Screenshots and Demo : will upload soon

Softwares: 
1) ESP-IDF
