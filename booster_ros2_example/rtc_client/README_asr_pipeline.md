# ROS2 ASR pipeline (audio capture + VAD + Riva)

This provides a modular, vendor-independent ASR pipeline:

1) `audio_capture_node.py` publishes PCM16 frames on `/audio/frames`
2) `vad_endpoint_node.py` adds pre-roll + hysteresis endpointing
3) `riva_asr_node.py` sends utterances to Riva and publishes `/asr/text`

## Nodes

### audio_capture_node.py
Captures audio via `arecord` (PCM16 mono).

```
python src/audio_capture_node.py --sample-rate 16000 --frame-ms 20
```

### vad_endpoint_node.py
Default VAD: WebRTC (if installed), else energy VAD.

```
python src/vad_endpoint_node.py \
  --input-topic /audio/frames \
  --utterance-topic /audio/utterance_audio \
  --start-topic /audio/utterance_start \
  --end-topic /audio/utterance_end
```

Suggested settings (robot):
- `--pre-roll-ms 400`
- `--speech-start-ms 240`
- `--speech-end-silence-ms 700`
- `--min-utterance-ms 400`
- `--max-utterance-ms 15000`

### riva_asr_node.py
Uses NVIDIA Riva client if installed (`riva.client`).

```
export RIVA_URI=localhost:50051
export RIVA_USE_SSL=0
export RIVA_API_KEY=""
python src/riva_asr_node.py --language en-US
```

Dynamic language switching:
```
ros2 param set /riva_asr language zh-CN
```

## Topics
- `/audio/frames` (UInt8MultiArray): PCM16 frames
- `/audio/utterance_start` (Empty)
- `/audio/utterance_audio` (UInt8MultiArray)
- `/audio/utterance_end` (Empty)
- `/asr/partial` (String) [reserved]
- `/asr/text` (String) final text

## Notes
- If `webrtcvad` is not installed, the VAD node falls back to RMS energy.
- Audio is fixed to 16 kHz mono to align with Riva defaults.
