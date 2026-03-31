#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_DIR="${TARGET_DIR:-/opt/booster/AI_chat_POC}"
PYTHON_BIN="${PYTHON_BIN:-python3}"
PIP_USER_FLAG="${PIP_USER_FLAG:---user}"
INSTALL_VOSK=0
SKIP_APT=0
VOSK_MODEL_URL="${VOSK_MODEL_URL:-https://alphacephei.com/vosk/models/vosk-model-small-en-us-0.15.zip}"
VOSK_MODEL_DIR="${VOSK_MODEL_DIR:-$HOME/.local/share/vosk}"

usage() {
  cat <<EOF
Usage: $(basename "$0") [--target-dir PATH] [--with-vosk] [--skip-apt]

  --target-dir PATH  Install bundle files under PATH.
  --with-vosk        Install the optional vosk Python package and download a model.
  --skip-apt         Skip apt package installation.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --target-dir)
      TARGET_DIR="$2"
      shift 2
      ;;
    --with-vosk)
      INSTALL_VOSK=1
      shift
      ;;
    --skip-apt)
      SKIP_APT=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 1
      ;;
  esac
done

if [[ $EUID -ne 0 && "$TARGET_DIR" == /opt/* ]]; then
  echo "Installing to $TARGET_DIR usually requires sudo." >&2
fi

if [[ "$SKIP_APT" -eq 0 ]]; then
  if command -v apt-get >/dev/null 2>&1; then
    sudo apt-get update
    sudo apt-get install -y \
      python3 \
      python3-pip \
      alsa-utils \
      ffmpeg \
      curl \
      unzip
  else
    echo "apt-get not found; skipping system package installation." >&2
  fi
fi

mkdir -p "$TARGET_DIR"
rsync -a \
  --exclude '__pycache__' \
  --exclude '*.pyc' \
  "$ROOT_DIR/" \
  "$TARGET_DIR/"

if [[ "$INSTALL_VOSK" -eq 1 ]]; then
  "$PYTHON_BIN" -m pip install $PIP_USER_FLAG vosk
  mkdir -p "$VOSK_MODEL_DIR"
  MODEL_ZIP="$VOSK_MODEL_DIR/$(basename "$VOSK_MODEL_URL")"
  MODEL_DIR="${MODEL_ZIP%.zip}"
  if [[ ! -d "$MODEL_DIR" ]]; then
    if command -v curl >/dev/null 2>&1; then
      curl -L "$VOSK_MODEL_URL" -o "$MODEL_ZIP"
    else
      wget -O "$MODEL_ZIP" "$VOSK_MODEL_URL"
    fi
    unzip -q "$MODEL_ZIP" -d "$VOSK_MODEL_DIR"
  fi
  echo "Vosk model installed under: $MODEL_DIR"
fi

cat <<EOF
Bundle copied to: $TARGET_DIR

Next:
1. Source the robot ROS environment that provides rclpy and booster_interface.
2. Edit $TARGET_DIR/config/config.json and add your API keys/endpoints.
3. Run $TARGET_DIR/run_llm_web_test.sh --input-mode text

If you want wakeword/realtime ASR, rerun this installer with --with-vosk and set:
  asr_model_path=$VOSK_MODEL_DIR/<model-directory>
  stt_realtime_enabled=true
EOF
