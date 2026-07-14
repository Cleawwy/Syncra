# Syncra

Syncra is a mobile robot digital twin prototype for an omnidirectional robot platform.

The project combines an NVIDIA Omniverse Kit application with a lightweight robot state pipeline. The current public code focuses on visualization, state handling, and safe telemetry-first integration.

## Current Status

- Omniverse Kit app: `digitaltwin.mobilebot`
- Robot visualization: three-wheel mobile robot placeholder scene
- Telemetry path: robot/mobilebot state into a digital twin state model
- Safety stage: telemetry-first; real robot movement commands are not part of the default public path

The physical platform uses FEETECH STS3215-12V bus servos through a Waveshare Bus Servo Adapter.

## Repository Layout

```text
kit-app-template/
  source/                 Omniverse Kit app and extension source
  tools/                  Kit template tooling
  repo.sh / repo.bat      Kit template entry points

source_local/
  source/                 local copy of the generated Kit app source
  mobilebot_control.py    mobile robot helper/control code

scripts/
  sample_mqtt_publisher.py
  twin_state_service.py
  check_http_state.py
  capture_evidence.py
  self_test_state_processing.py
  real_servo_scan.py
  real_servo_mqtt_bridge.py
```

## License

The NVIDIA Kit template license and product terms are included under `kit-app-template/`.
