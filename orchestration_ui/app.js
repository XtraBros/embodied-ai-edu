const state = {
  apiBase: "",
  connected: false,
  move: { vx: 0, vy: 0, vyaw: 0 },
  sendTimer: null,
};

const statusConnection = document.getElementById("status-connection");
const statusConnectionInline = document.getElementById("status-connection-inline");
const agentName = document.getElementById("agent-name");
const moveReadout = document.getElementById("move-readout");
const rotateReadout = document.getElementById("rotate-readout");
const headReadout = document.getElementById("head-readout");

const apiHost = document.getElementById("api-host");
const connectBtn = document.getElementById("btn-connect");
const stopBtn = document.getElementById("btn-stop");

function setStatus(text, ok) {
  statusConnection.textContent = text;
  statusConnection.style.background = ok
    ? "linear-gradient(135deg, rgba(61,214,167,0.6), rgba(248,111,68,0.3))"
    : "rgba(255,255,255,0.06)";
  if (statusConnectionInline) {
    statusConnectionInline.textContent = text;
  }
}

async function apiFetch(path, body) {
  const url = `${state.apiBase}${path}`;
  const res = await fetch(url, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body || {}),
  });
  if (!res.ok) {
    throw new Error(`Request failed: ${res.status}`);
  }
  return res.json();
}

connectBtn.addEventListener("click", async () => {
  const host = apiHost.value.trim();
  state.apiBase = host || "";
  try {
    await apiFetch("/api/ping");
    setStatus("Connected", true);
    state.connected = true;
  } catch (err) {
    setStatus("Disconnected", false);
    state.connected = false;
  }
});

stopBtn.addEventListener("click", async () => {
  await apiFetch("/api/stop");
  state.move = { vx: 0, vy: 0, vyaw: 0 };
  moveReadout.textContent = "vx 0.00 / vy 0.00";
  rotateReadout.textContent = "vyaw 0.00";
});

function startMoveLoop() {
  if (state.sendTimer) return;
  state.sendTimer = setInterval(() => {
    if (!state.connected) return;
    apiFetch("/api/move", state.move).catch(() => setStatus("Disconnected", false));
  }, 120);
}

function updateMoveReadout() {
  moveReadout.textContent = `vx ${state.move.vx.toFixed(2)} / vy ${state.move.vy.toFixed(2)}`;
  rotateReadout.textContent = `vyaw ${state.move.vyaw.toFixed(2)}`;
}

function setupJoystick(element, type) {
  const handle = element.querySelector(".joystick-handle");
  const radius = element.clientWidth / 2;
  const maxDist = radius - handle.clientWidth / 2 - 4;
  let active = false;

  const setHandle = (x, y) => {
    handle.style.transform = `translate(${x}px, ${y}px)`;
  };

  const reset = () => {
    active = false;
    handle.style.transform = "translate(-50%, -50%)";
    if (type === "move") {
      state.move.vx = 0;
      state.move.vy = 0;
    }
    if (type === "rotate") {
      state.move.vyaw = 0;
    }
    updateMoveReadout();
  };

  const onMove = (clientX, clientY) => {
    const rect = element.getBoundingClientRect();
    const dx = clientX - (rect.left + rect.width / 2);
    const dy = clientY - (rect.top + rect.height / 2);
    const dist = Math.min(Math.hypot(dx, dy), maxDist);
    const angle = Math.atan2(dy, dx);
    const x = Math.cos(angle) * dist;
    const y = Math.sin(angle) * dist;
    handle.style.transform = `translate(${x - handle.clientWidth / 2}px, ${y - handle.clientWidth / 2}px)`;

    const normX = x / maxDist;
    const normY = y / maxDist;
    if (type === "move") {
      state.move.vx = -normY * 0.6;
      state.move.vy = normX * 0.6;
    } else {
      state.move.vyaw = normX * 0.8;
    }
    updateMoveReadout();
  };

  element.addEventListener("pointerdown", (event) => {
    active = true;
    element.setPointerCapture(event.pointerId);
    onMove(event.clientX, event.clientY);
    startMoveLoop();
  });

  element.addEventListener("pointermove", (event) => {
    if (!active) return;
    onMove(event.clientX, event.clientY);
  });

  element.addEventListener("pointerup", reset);
  element.addEventListener("pointercancel", reset);
}

[...document.querySelectorAll(".joystick")].forEach((stick) => {
  setupJoystick(stick, stick.dataset.stick);
});

const headMap = {
  up: { pitch: 1, yaw: 0 },
  down: { pitch: -1, yaw: 0 },
  left: { pitch: 0, yaw: -1 },
  right: { pitch: 0, yaw: 1 },
  center: { pitch: 0, yaw: 0 },
  "left-up": { pitch: 1, yaw: -1 },
  "right-up": { pitch: 1, yaw: 1 },
  "left-down": { pitch: -1, yaw: -1 },
  "right-down": { pitch: -1, yaw: 1 },
};

[...document.querySelectorAll(".pad-btn")].forEach((btn) => {
  btn.addEventListener("click", async () => {
    const key = btn.dataset.head;
    const dir = headMap[key];
    if (!dir) return;
    headReadout.textContent = `pitch ${dir.pitch} / yaw ${dir.yaw}`;
    await apiFetch("/api/rotate_head_dir", dir);
  });
});

[...document.querySelectorAll("[data-mode]")].forEach((btn) => {
  btn.addEventListener("click", async () => {
    const mode = Number(btn.dataset.mode);
    await apiFetch("/api/change_mode", { mode });
  });
});

const actionMap = {
  wave: () => apiFetch("/api/wave_hand", { hand_index: 1, hand_action: 0 }),
  shake: () => apiFetch("/api/handshake", { hand_action: 0 }),
  pushup: () => apiFetch("/api/push_up"),
  liedown: () => apiFetch("/api/lie_down"),
  getup: () => apiFetch("/api/get_up"),
  "dance-newyear": () => apiFetch("/api/dance", { dance_id: 0 }),
  "dance-nezha": () => apiFetch("/api/dance", { dance_id: 1 }),
  "dance-towards": () => apiFetch("/api/dance", { dance_id: 2 }),
  "gesture-dabbing": () => apiFetch("/api/dance", { dance_id: 3 }),
  "gesture-ultraman": () => apiFetch("/api/dance", { dance_id: 4 }),
  "gesture-respect": () => apiFetch("/api/dance", { dance_id: 5 }),
  "gesture-cheer": () => apiFetch("/api/dance", { dance_id: 6 }),
  "gesture-luckycat": () => apiFetch("/api/dance", { dance_id: 7 }),
  "dance-stop": () => apiFetch("/api/dance", { dance_id: 1000 }),
};

[...document.querySelectorAll(".action-btn")].forEach((btn) => {
  btn.addEventListener("click", () => {
    const key = btn.dataset.action;
    const action = actionMap[key];
    if (action) {
      action();
    }
  });
});

const btnGesture = document.getElementById("btn-gesture");
btnGesture.addEventListener("click", () => apiFetch("/api/gesture_trigger"));

const btnAgentToggle = document.getElementById("btn-agent-toggle");
const btnSpeak = document.getElementById("btn-speak");
const btnVoice = document.getElementById("btn-voice");
const voiceText = document.getElementById("voice-text");
const voiceStatus = document.getElementById("voice-status");
const btnBack = document.getElementById("btn-back");
const speechSupported = "webkitSpeechRecognition" in window;
let agentActive = false;

function setAgentState(active) {
  agentActive = active;
  if (btnAgentToggle) {
    btnAgentToggle.textContent = active ? "Deactivate" : "Activate";
  }
  if (agentName) {
    agentName.textContent = active ? "Active" : "Idle";
  }
  if (btnVoice) {
    btnVoice.disabled = !active;
  }
  if (voiceStatus) {
    voiceStatus.textContent = speechSupported
      ? active
        ? "Idle"
        : "Activate agent first"
      : "Speech recognition not supported";
  }
}

async function startAgent() {
  const systemPrompt = document.getElementById("system-prompt").value;
  const welcomeMessage = document.getElementById("welcome-message").value;
  await apiFetch("/api/ai/start", { system_prompt: systemPrompt, welcome_message: welcomeMessage });
  setAgentState(true);
}

async function stopAgent() {
  await apiFetch("/api/ai/stop");
  setAgentState(false);
}

if (btnAgentToggle) {
  btnAgentToggle.addEventListener("click", async () => {
    if (agentActive) {
      await stopAgent();
    } else {
      await startAgent();
    }
  });
}

if (btnBack) {
  btnBack.addEventListener("click", () => {
    window.history.back();
  });
}

btnSpeak.addEventListener("click", () => {
  const msg = document.getElementById("speak-text").value;
  apiFetch("/api/ai/speak", { msg });
});

function extractResponseText(raw) {
  if (!raw) return "";
  let cleaned = String(raw).trim();
  if (cleaned.startsWith("```")) {
    cleaned = cleaned.replace(/^```/g, "").replace(/```$/g, "").trim();
    if (cleaned.toLowerCase().startsWith("json")) {
      cleaned = cleaned.slice(4).trim();
    }
  }
  try {
    const payload = JSON.parse(cleaned);
    if (payload && typeof payload === "object" && "response" in payload) {
      const value = payload.response;
      return typeof value === "string" ? value : JSON.stringify(value);
    }
  } catch (err) {
    if (cleaned.includes('"response"')) {
      const start = cleaned.indexOf('"response"');
      const slice = cleaned.slice(start);
      const match = slice.match(/"response"\s*:\s*"([^"]*)"/);
      if (match && match[1]) return match[1];
    }
  }
  return cleaned;
}

async function sendVoiceToLlm(text) {
  if (!agentActive) {
    voiceStatus.textContent = "Activate agent first";
    return;
  }
  const trimmed = text.trim();
  if (!trimmed) return;
  voiceStatus.textContent = "Thinking...";
  try {
    const result = await apiFetch("/api/llm/query", { text: trimmed });
    const responseText = extractResponseText(result.response || "");
    voiceStatus.textContent = "Speaking...";
    if (responseText) {
      await apiFetch("/api/ai/speak", { msg: responseText });
    }
    voiceStatus.textContent = "Idle";
  } catch (err) {
    voiceStatus.textContent = "Error";
  }
}

let speechRecognition = null;
let voiceActive = false;
let voiceTimer = null;
let finalTranscript = "";
const voiceAutoStopMs = 1200;

function setVoiceStatus(text, active = false) {
  voiceStatus.textContent = text;
  btnVoice.textContent = active ? "Stop" : "Mic";
}

function scheduleVoiceStop() {
  clearTimeout(voiceTimer);
  voiceTimer = setTimeout(() => {
    if (voiceActive) {
      speechRecognition.stop();
    }
  }, voiceAutoStopMs);
}

if (speechSupported) {
  speechRecognition = new webkitSpeechRecognition();
  speechRecognition.continuous = true;
  speechRecognition.interimResults = true;
  speechRecognition.lang = "en-US";

  speechRecognition.onstart = () => {
    finalTranscript = "";
    voiceText.value = "";
    setVoiceStatus("Listening...", true);
  };

  speechRecognition.onerror = (event) => {
    setVoiceStatus(`Error: ${event.error}`, false);
    voiceActive = false;
  };

  speechRecognition.onend = () => {
    voiceActive = false;
    setVoiceStatus("Idle", false);
    const text = finalTranscript.trim();
    if (text) {
      sendVoiceToLlm(text);
    }
  };

  speechRecognition.onresult = (event) => {
    let interimTranscript = "";
    for (let i = event.resultIndex; i < event.results.length; ++i) {
      const chunk = event.results[i][0].transcript;
      if (event.results[i].isFinal) {
        finalTranscript += chunk;
      } else {
        interimTranscript += chunk;
      }
    }
    voiceText.value = finalTranscript + interimTranscript;
    scheduleVoiceStop();
  };

  btnVoice.addEventListener("click", () => {
    if (!speechRecognition) return;
    if (voiceActive) {
      speechRecognition.stop();
      return;
    }
    voiceActive = true;
    speechRecognition.start();
  });
} else {
  setVoiceStatus("Speech recognition not supported", false);
  btnVoice.disabled = true;
}

setAgentState(false);

const btnGestureEnable = document.getElementById("btn-gesture-enable");
const btnGestureDisable = document.getElementById("btn-gesture-disable");

btnGestureEnable.addEventListener("click", () => apiFetch("/api/gesture/enable"));
btnGestureDisable.addEventListener("click", () => apiFetch("/api/gesture/disable"));

const streamFeed = document.getElementById("stream-feed");
const streamUrl = document.getElementById("stream-url");
const btnSetStream = document.getElementById("btn-set-stream");

btnSetStream.addEventListener("click", () => {
  const url = streamUrl.value.trim();
  if (!url) return;
  streamFeed.src = url;
  streamFeed.onload = () => {
    const overlay = document.querySelector(".video-overlay .overlay-sub");
    overlay.textContent = "Live stream connected";
  };
});
