Orchestration UI TODO
- Merge other-device changes for slideshow + thumbnails and verify UI parity.
- [x] Add Export Settings UI:
  - [x] Button A: download raw track JSON (array of {ts, a} in tenths of degrees).
  - [x] Button B: SSH publish flow (user@ip + password).
- [x] SSH publish flow steps:
  - [x] Generate orch_id + track_id (32-char hex, MD5-like).
  - [x] Create `/opt/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations/<orch_id>/track_<track_id>/`.
  - [x] Write `<track_id>.json` with raw frame array.
  - [x] Patch `/opt/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations.json` with new entry.
- [ ] Confirm default `track_type` to use (backup shows `track_type = 1`; app defaults to 0 when missing).

Session findings (paths + formats)
- On-robot storage base: `/opt/BoosterAgent/agent_storage/com.boosterobotics.teaching/`.
- Orchestration index file:
  - `/opt/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations.json`
  - Example in backup: `robot_opt/opt_booster_backup/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations.json`
  - Structure: `{ "orch_list": [ { id, name, icon_name, activated, shortcut, tracks: [ { track_id, track_type } ] } ] }`
- Track data file layout:
  - `/opt/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations/<orch_id>/track_<track_id>/<track_id>.json`
  - Example in backup: `robot_opt/opt_booster_backup/BoosterAgent/agent_storage/com.boosterobotics.teaching/orchestrations/c89b128f503cfa7326c923b2d3b66008/track_c20e75a59b5ccbd14c94a931c3a934db/c20e75a59b5ccbd14c94a931c3a934db.json`
  - Format: array of frames: `[{"a":[...], "ts": 510}, ...]` where `a` values are tenths of degrees.
- App upload operation (if later needed via network):
  - `orchestration_operation` endpoint, operation `orch_set_track_data`.
  - Payload fields: `agent_id`, `operation`, `orch_id`, `track_id`, `track_type`, `content` (list of joints records).
- App uses existing track metadata:
  - `track_id` + `track_type` come from `OrchInfo.tracks[0]` when opening editor.
