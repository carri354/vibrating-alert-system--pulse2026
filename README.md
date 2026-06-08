# Vibrating Alert System (Pulse 2026)

## Goal
Alert deaf people about fire alarms using vibration rather than sound

## System Overview
1) Audio is captured through an electret microphone. The signal is amplified and sent to an Arduino UNO.
2) The Arduino runs a real-time FFT algorithm to transform audio signal into frequencies
3) The Arduino looks through frequencies to detect fire alarm frequency (3150 Hz)
4) If an alarm is detected, we trigger a flash an LED and activate a DC motor to create a vibration

## Results
After working together for 10 hours, our project won 3rd place in the Pulse 2026 Hardware Competition at UIUC (https://www.pulseillinois.org/)


https://github.com/user-attachments/assets/b1d79040-0d1d-4cd2-af90-08fefc9975bd

Credit: Michael Bauer, Carlos Arriola, Kevin Shi, Marc Solc
