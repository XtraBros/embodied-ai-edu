# LLM Web Test Package

This folder packages the minimum assets needed to run `main.py` on a robot without shipping the entire ROS workspace.

## What is included

- `src/main.py`: main terminal entrypoint
- `src/llm_web_client.py`: LLM HTTP client
- `src/stt_openai.py`: OpenAI speech-to-text path
- `src/tts_openai.py`: OpenAI text-to-speech path
- `src/music_player.py`: local audio playback helper
- `src/keyboard_joystick.py`: joystick helper used by some AI-chat flows
- `config/config.json`: sanitized sample config
- `config/action_commands.json`: built-in action/button mappings
- `config/song_library.json`: optional song map
- `install_on_robot.sh`: robot-side install helper
- `run_llm_web_test.sh`: wrapper that sources common ROS environments, then launches the Python entrypoint

## Dependency check

`main.py` depends on:

- Python standard library modules only for HTTP, JSON, subprocess, temp files, and playback control
- ROS 2 Python runtime: `rclpy`
- Booster ROS interfaces in the active ROS environment: `booster_interface.msg` and `booster_interface.srv`
- A running robot RPC service: `booster_rtc_service`
- ALSA tools:
  - `arecord` for microphone capture in ASR mode
  - `aplay` for local TTS playback when `tts_play_command` is `aplay`
- Network access to your configured LLM/STT/TTS backend

Optional dependency:

- `vosk` plus a local Vosk model if you want realtime wakeword/transcript gating with `stt_realtime_enabled=true`

## Important limitation

This bundle does **not** ship `rclpy`, `booster_interface`, or `booster_rtc_service`.

Those must already exist on the robot, typically through the robot's ROS 2 image or an installed Booster workspace. The wrapper script tries common setup files, but if your robot uses a different overlay you should export:

```bash
export BOOSTER_ROS_SETUP=/path/to/setup.bash
```

before launching.

## Install on robot

Copy this folder to the robot, then run:

```bash
cd AI_chat_POC
./install_on_robot.sh
```

That installs the basic OS packages, copies the bundle to `/opt/booster/AI_chat_POC`, and prints the next steps.

If you want realtime Vosk-assisted ASR:

```bash
./install_on_robot.sh --with-vosk
```

## Configure

Edit:

`config/config.json`

Required fields depend on your backend:

- OpenAI:
  - `provider`: `openai`
  - `openai_api_key`
  - optionally `model`, `stt_model`, `tts_model`, `tts_voice`
- RAGFlow agent:
  - `provider`: `ragflow`
  - `api_base`
  - `agent_id`
  - `ragflow_api_key`

Recommended first-run settings:

- Keep `stt_realtime_enabled=false`
- Keep `stt_trigger_required=false`
- Start with `--input-mode text`

After text mode is stable, enable ASR if needed.

## Run

Text mode:

```bash
/opt/booster/AI_chat_POC/run_llm_web_test.sh --input-mode text
```

ASR mode:

```bash
/opt/booster/AI_chat_POC/run_llm_web_test.sh --input-mode asr
```

If you want the robot AI-chat pipeline enabled:

```bash
/opt/booster/AI_chat_POC/run_llm_web_test.sh --ai-chat --input-mode text
```

## Operational notes

- If `booster_rtc_service` is not running, the script will wait and not proceed.
- If `rclpy` or `booster_interface` are missing from the sourced environment, startup will fail before the terminal loop begins.
- If `arecord` is missing, ASR mode will fail.
- If `aplay` is missing, TTS playback will fail unless you change `tts_play_command`.
- The sample config included here is scrubbed. Fill in your own endpoints and credentials before sharing or deploying.

## `main.py` flags and settings

CLI flags:

- `--ai-chat`: enables the robot AI chat pipeline by calling the robot RPC service before entering the input loop
- `--input-mode text`: reads prompts from the terminal
- `--input-mode asr`: records microphone input and runs ASR before sending text to the LLM
- `--config <path>`: uses a different JSON config file instead of `config/config.json`
- `--no-tts`: disables spoken output even if `enable_tts` is true in the config

Example:

```bash
python3 src/main.py --config config/config.json --input-mode text
```

Main config settings:

- LLM:
  - `provider`: `openai` or `ragflow`
  - `api_base`: base URL for the selected backend
  - `openai_api_key` or `ragflow_api_key`: backend credential
  - `agent_id`: required for `ragflow`
  - `model`: LLM model name
  - `request_timeout_sec`: HTTP timeout
  - `stream_llm`: enables streamed LLM output
  - `response_field`: field to read from a JSON LLM response
  - `stream_response_json`: extracts only the response field during streaming
  - `ignore_action`: ignores returned action commands

- STT and ASR:
  - `stt_backend`: `openai` or `vosk`
  - `stt_api_base`: STT endpoint base URL
  - `stt_model`: transcription model
  - `stt_realtime_enabled`: enables the Vosk-assisted realtime capture path inside `stt_openai.py`
  - `stt_trigger_required`: requires the trigger phrase before returning text
  - `stt_trigger_phrase`: wake phrase text
  - `stt_trigger_timeout_sec`: max time to wait for the trigger phrase
  - `stt_sample_rate`, `stt_max_sec`, `stt_silence_ms`, `stt_rms_threshold`, `stt_min_speech_ms`: microphone capture tuning
  - `asr_model_path`: local Vosk model path when using Vosk features
  - `asr_sample_rate`, `asr_chunk_ms`, `asr_max_sec`: classic Vosk capture settings

- TTS:
  - `enable_tts`: master TTS enable switch
  - `tts_api_base`: TTS endpoint base URL
  - `tts_model`: TTS model name
  - `tts_voice`: TTS voice
  - `tts_response_format`: usually `wav`
  - `tts_play_command`: local playback command, default `aplay`
  - `tts_keep_audio`: keeps generated audio files instead of deleting them

- Robot RPC and ROS:
  - `joystick_topic`: topic used for `RemoteControllerState`
  - `enter_booster_agent_mode`: optionally sends the LT+RT+A combo on startup
  - `joystick_script`: alternate joystick helper path
  - `joystick_event_press`, `joystick_event_release`, `joystick_event_axis`, `joystick_event_axis_release`: event codes for button and axis commands
  - `joystick_hold_sec`: button hold duration

- Actions and motion:
  - `enable_builtin_commands`: enables built-in button-combo actions from `action_commands.json`
  - `walk_step_sec`: duration per scheduled walk step
  - `walk_magnitude`: walk axis magnitude
  - `auto_stop_sec`: default time before auto-releasing a triggered action
  - `loco_action_runner`: external action runner command for non-built-in actions
  - `loco_interface`: interface argument passed to the action runner

- Orchestration and media:
  - `song_library_path`: alternate song library JSON path
  - `action_commands_path`: alternate action command JSON path
  - `music_player_script`: alternate path for the music helper
  - `orch_metadata_path`, `orch_storage_dir`: orchestration metadata used to estimate shortcut duration
  - `orch_duration_fallback_sec`, `orch_duration_padding_sec`: fallback orchestration timing

Behavior notes:

- `--input-mode text` with `enable_tts=true` is the simplest first-run path.
- `--input-mode asr` still requires `arecord` even when using OpenAI STT.
- `--ai-chat` requires the robot-side `booster_rtc_service` to be available.
