# Booster Web Pilot

A lightweight web UI that mirrors the Booster Pilot app controls and calls ROS2 RPC services.

## Run locally (robot)
```bash
cd /opt/booster/web_ui
python server.py
```

Open `http://<robot-ip>:8000` in a browser.

## Gesture trigger on speech
- The server subscribes to `subtitle` and runs `k1_gesture_001` when the robot finishes speaking.
- Override the topic or port with env vars:
```bash
export SUBTITLE_TOPIC=subtitle
export WEB_PILOT_PORT=8000
python server.py
```
- Override the gesture command if needed:
```bash
export K1_GESTURE_CMD="python /opt/booster/booster_gym/deploy/deploy.py --config=k1_gesture_001.yaml --auto-start --duration=6"
python server.py
```

## Notes
- AI chat uses `booster_rtc_service` and calls Start/Speak/Stop.
- Movement and actions use `booster_rpc_service` and Loco API IDs.
