# Telemetry database

Everything the car publishes is recorded into a local database on the PC,
shaped like DynamoDB so the same tables can move to AWS later unchanged.

```bash
tools/telemetry/start_telemetry.sh
```

This uses the MQTT broker already running on the PC (`start_ui.sh` or
`start_broker.sh`), or starts one if none is. It then imports any saved
console logs in `logs/` not imported yet, collects live into
`telemetry/picocar.db`, and opens http://127.0.0.1:8081. Ctrl-C stops it;
the database stays.

Settings (environment variables): `UI_PORT` (8081), `MQTT_PORT` (1883),
`DB`, `NO_IMPORT=1` (skip `logs/`), `NO_OPEN=1` (don't open the browser).
Standard library only: Python 3 with its built-in SQLite. No internet needed.

## The page

- **Live**: the segment being recorded, its running statistics, the last
  60 s of wheel speeds, line error and turn rate, and the latest events.
- **Runs**: every run segment, filterable (course attempts, standby,
  outcome, search by label, ID or barcode letter) and sortable. Opening
  one shows:
  - its statistics and the time spent in each state;
  - charts on a shared time axis with the car's state as a band above
    them. Hovering shows every value at that moment, and the charts can
    also be shown as a table;
  - events filterable by type, and the car log with search;
  - a label and notes you can edit, CSV/JSON export, and delete.
- **Explore tables**: query the tables like DynamoDB. Query (partition key
  and sort key condition, on the table or the `by_type` index) or Scan,
  with filters, Limit, order and paging. Each request also shows the
  equivalent `aws dynamodb` CLI command.

## Run segments

The collector splits the car's time into run segments:

| Kind | Starts | Ends (outcome) |
|---|---|---|
| course | the car starts driving (S2/S9 to S3), or the first record already shows it driving | `finished` (S9), `fault: <critical faults>` (S10), `reset` (back to S1/S2), `offline` (no telemetry for 10 s or status offline) |
| standby | the car is online and not on a course | `course started`, `offline` |

A run left open by a crash is closed as `interrupted` at its last
record when the collector next starts. One stopped with Ctrl-C ends as
`collector stopped`.

## Tables

| Table | Partition key | Sort key | Index | Holds |
|---|---|---|---|---|
| PicoCarRuns | `run_id` (S) | - | - | one item per run segment, summary statistics updated live |
| PicoCarTelemetry | `run_id` (S) | `ts` (N, ms since 1970) | - | every telemetry record (5 per second) |
| PicoCarEvents | `run_id` (S) | `ts_seq` (S, `<13-digit ms>#<seq>`) | `by_type`: `type` + `ts_seq` | what happened, one item per event |
| PicoCarLogs | `run_id` (S) | `ts_seq` (S) | - | every car log line |

### PicoCarTelemetry attributes

Every field of the 24-byte version 2 record (docs/MQTT.md): `version seq
mission_state motion_state speed_left_mms speed_right_mms enc_left
enc_right line_state line_error last_barcode reject_reason hump_max_mm
obstacle_width_mm obstacle_closest_mm obstacle_bearing_x10deg bypass_plan
distance_mm motion_class turn_rate_dps faults scan_state pitch_x10
roll_deg hump_now_mm` (version 1 records from older firmware read 0 for
the last three).

Added by the collector:
- names: `time` (local, readable), `mission_name`, `motion_name`,
  `line_name`, `fault_names`, `faults_critical`;
- derived: `speed_mean_mms`, `speed_diff_mms` (right minus left),
  `dt_ms` (since the previous record), `lost_before` (records missed
  before this one, from the sequence number);
- `raw_hex`: the bytes as received.

### PicoCarEvents types

| type | When | Attributes |
|---|---|---|
| `run_start`, `run_end` | segment boundaries | `kind`, `from_state` / `outcome` |
| `first_record` | first telemetry of a segment | `state`, `faults` |
| `state` | mission state changed | `from_state`, `to_state`, `distance_mm` |
| `fault_set`, `fault_cleared` | a fault bit changed | `fault`, `critical`, `meaning`, `state` |
| `line_lost`, `line_found` | line state LOST entered / left | `distance_mm`, `line_error` |
| `barcode` | letter decoded | `letter`, `meaning`, `text` |
| `barcode_reject` | decode failed | `reason`, `why`, `elements`, `text` |
| `obstacle`, `obstacle_plan` | obstacle seen / bypass chosen | `text` / `plan`, `width_mm`, `closest_mm`, `bearing_deg` |
| `hump` | new hump peak | `peak_mm` |
| `impact` | motion class "impact" | `state` |
| `command` | a command byte on `p5is20/car/cmd` (from any console) | `command`, `cmd_seq`, `via` |
| `command_ack` | the car's `CMD ... received` line | `command`, `cmd_seq`, `result` |
| `console_command` | a line typed in the console (imported logs) | `line` |
| `drive_key` | manual drive key changed | `key` |
| `link` | car status topic changed | `status` |
| `packet_loss` | records missed (at most one event per 5 s) | `records`, `since_ms` |
| `telemetry_gap` | no telemetry for 3-10 s | `seconds` |
| `calibration`, `scan`, `boot`, `manual_drive`, `log_gap`, `problem` | matching car log lines | `text` |

### PicoCarRuns attributes

| Group | Attributes |
|---|---|
| Identity and timing | `run_id`, `kind`, `source` (live/import), `started`, `ended` (+ `_iso`), `duration_s`, `outcome` |
| Your annotations | `label`, `notes` (editable on the page) |
| Volume | `records`, `records_lost`, `telemetry_gaps`, `events`, `log_lines`, `commands` |
| Motion | `distance_mm`, `speed_mean_mms` (while on the course), `speed_max_mms`, `turn_rate_abs_max_dps` |
| Line following | `line_err_rms`, `line_err_abs_mean`, `line_err_abs_max`, `line_lost`, `searches` |
| Mission | `barcodes` (letters in order), `barcode_rejects`, `obstacles`, `hump_max_mm`, `faults_seen`, `states_visited`, `state_time_s` (seconds per state), `first_state`, `last_state` |

## Importing saved logs

The console saves `logs/car_<date>_<time>.csv` (telemetry) and `.log`
(car log lines, console notices, typed commands); the launcher replays
them through the same collector:
- **Change detection:** each file is imported once, keyed by its
  modification time. A file that grew is re-imported, replacing its
  earlier runs.
- **No duplicates:** a file whose time span the collector already
  recorded live is skipped, so the same run is never stored twice.
- **Timing:** imported car log lines carry the time of the console
  notice before them, to the second.

To import by hand, or browse without collecting:

```bash
python3 tools/telemetry/telemetry_db.py --import logs --no-collect
```

## Moving to AWS later

The tables, keys and the `by_type` index are the DynamoDB design. Items
are plain JSON and the queries are Query/Scan with key conditions and
filters, so:
- create the four tables with the keys above;
- point an IoT Core rule (or a Lambda doing what `Collector` does) at
  them;
- export a run as JSON from the page to backfill.
