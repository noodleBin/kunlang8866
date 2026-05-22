# Audio

## Introduction
  The Audio module detect the siren sound coming from the active emergency vehicle. It detects and output siren on/off status, moving status and the relative position of the siren. When active emergency vehicles detected, you can also see active alerts on Dreamview.


## Input
  * Audio signal data (cyber channel `/century/sensor/microphone`)

## Output
  * Audio detection result, including the siren active/inactive status, moving status(approaching/departing/stationary), and the positions (cyber channel `century/audio/audio_detection`).
