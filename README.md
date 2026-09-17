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

## Versions

| Version | Change |
| --- | --- |
| 4 | `request_id` in `HeaderInfo_t` and `AckInfo_t`; `upload_id`/`tag_count`/`tag_index` on the tag upload set |
| 3 | `antenna_id`, revisit request, `confirmed` flag; drop `measurement_k` |
| 2 | Python pulse message, `rate_state`, `measurement_k` |
