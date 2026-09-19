# TagTrackerTunnelProtocol

This is the protocol definition for the mavlink communication between QGC TagTracker and the controller running on the rPi.

## Request ids and retries

Every GCS → controller command carries `HeaderInfo_t::request_id`. The GCS
assigns a fresh value for each new command and reuses it, byte-for-byte, on
every retry of that command after a lost ACK. The controller remembers the
ACK it sent for recent request ids and replays it for a repeat instead of
executing the command again; a repeat id with a different command is NACKed.
`AckInfo_t::request_id` echoes the id so the GCS can drop stale ACKs.

Controller-originated messages (heartbeat, pulses, collection status, bearing
result) set `request_id = 0`.

## Tag upload set

`START_TAGS`, `TAG` … `END_TAGS` form one set identified by `upload_id`.
`START_TAGS` announces `tag_count`; each `TAG` carries its `tag_index`;
`END_TAGS` restates both. The controller NACKs `END_TAGS` with
`incomplete: missing i,j` until every index has arrived, and NACKs any `TAG`
or `END_TAGS` whose `upload_id` does not match the open bracket.

## Operation progress

The ACK for a long-running command (`START_DETECTION`, `STOP_DETECTION`,
`RAW_CAPTURE`, `SAVE_LOGS`, `CLEAN_LOGS`) means the command was accepted.
The work itself is reported by `OPERATION_PROGRESS` messages carrying the
originating `command`/`request_id`, a `state` (running / complete / failed),
`step` of `step_count` (0 = indeterminate) and a short message. The
controller sends one on begin, on each step change and on finish, and
re-sends the current state about once a second while running. Only one
operation runs at a time; another long-running command arriving meanwhile is
NACKed with `Busy: <operation> in progress`.

## Versions

| Version | Change |
| --- | --- |
| 5 | `OPERATION_PROGRESS` message for long-running commands |
| 4 | `request_id` in `HeaderInfo_t` and `AckInfo_t`; `upload_id`/`tag_count`/`tag_index` on the tag upload set |
| 3 | `antenna_id`, revisit request, `confirmed` flag; drop `measurement_k` |
| 2 | Python pulse message, `rate_state`, `measurement_k` |
