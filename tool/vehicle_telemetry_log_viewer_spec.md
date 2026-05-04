# Vehicle Telemetry Log Viewer Specification

## 1. Purpose

This document defines the requirements and behavior of a browser-based viewer for vehicle telemetry logs.

The viewer is intended to decode, inspect, summarize, and graph telemetry data generated from the vehicle telemetry protocol. It should support development-time workflows where logs are copied from an ESP32 serial monitor, as well as binary log files produced by the child unit.

The goals of this specification are:

- Keep viewer behavior consistent across implementations.
- Support pasted serial log text as the primary workflow.
- Support binary log files for future direct log import.
- Display decoded records in a scrollable table.
- Display file and telemetry summary information.
- Plot telemetry values over time.
- Allow linked selection between table rows and graph points.
- Preserve forward compatibility with future protocol revisions where possible.

---

## 2. Target Platform

The viewer must be designed to run in a web browser.

Requirements:

- Must run without requiring a specific OS.
- Must accept local file input.
- Must accept direct text paste input.
- Must not require server-side processing.
- Must not upload telemetry data to a server.
- Must work with files or pasted text entirely on the local client.

The specific rendering library, frontend framework, or implementation technology is intentionally out of scope for this specification.

---

## 3. Related Data Specifications

This viewer is based on the following telemetry data concepts:

- Transport payload size: 29 bytes
- Application data size: 13 bytes
- LogRecord V1 size: 24 bytes
- Optional File Header
- Optional Summary Block

The viewer must be able to decode at least the following application data type:

```text
0x01 = Vehicle Telemetry V1
```

---

## 4. Supported Input Methods

The viewer must support two input methods:

1. File input
2. Text paste input

### 4.1 File Input

The viewer must accept at least:

| Extension | Meaning |
|---|---|
| `.bin` | Binary log data |
| `.txt` | Text log data, serial log, hex dump, or CSV-like text |

The extension should be used only as a hint. Actual parsing must be based on content auto-detection where possible.

### 4.2 Text Paste Input

The viewer must provide a text area where the user can paste log text directly.

Primary expected text input:

- ESP32 serial monitor output

Secondary accepted text input:

- Hex dump text
- Whitespace-separated hex bytes
- Comma-separated hex bytes
- `0x` prefixed hex values
- Multiple records separated by newlines
- Lines containing comments

---

## 5. Supported Input Data Types

The viewer must support or attempt to support the following data types:

| Input Data Type | Priority | Description |
|---|---:|---|
| Full binary log | 1 | File Header + LogRecord[] + optional Summary Block |
| LogRecord V1 sequence | 2 | Raw sequence of 24-byte records |
| Raw `appData[13]` sequence | 3 | Sequence of 13-byte application data records |
| Text / Serial log | 4 | ESP32 serial output, hex dumps, CSV-like data |

Although serial log paste is expected to be the most common practical workflow, auto-detection should still follow the priority order above.

---

## 6. Input Auto-Detection

The viewer must attempt to determine input type automatically.

Recommended detection order:

```text
1. If first 4 bytes are "SCUB", parse as Full binary log.
2. Else, if binary length is divisible by 24, parse as LogRecord V1 sequence.
3. Else, if binary length is divisible by 13, parse as raw appData[13] sequence.
4. Else, if input is text, attempt text / serial log parsing.
5. Else, report parse failure.
```

### 6.1 Serial Log Priority

Because serial log paste is expected to be the main workflow, text input parsing must be permissive.

The parser should be able to extract useful payloads from lines such as:

```text
RX ok: seq=41 sender=0x0001 appCounter=41 txMillis=42437 rssi=-78 dBm totalRx=41 totalLost=0
Payload: E2 01 00 01 00 00 00 29 00 00 A5 C5 00 29 01 01 02 6F 00 30 FF 80 00 80 00 00 00 12 34
```

The parser should prefer explicitly labeled payload lines when available.

---

## 7. Text Parsing Rules

The text parser must accept the following input patterns.

### 7.1 Whitespace-Separated Hex

Allowed, although not expected to be common.

Example:

```text
01 01 02 6F 00 30 FF 80 00 80 00 00 00
```

### 7.2 Newline-Separated Records

Allowed, although not expected to be common.

Example:

```text
01 01 02 6F 00 30 FF 80 00 80 00 00 00
01 01 02 71 00 31 FF 80 00 80 00 00 00
```

### 7.3 `0x` Prefix

Allowed.

Example:

```text
0x01 0x01 0x02 0x6F 0x00 0x30 0xFF 0x80 0x00 0x80 0x00 0x00 0x00
```

### 7.4 Comma-Separated Hex

Allowed.

Example:

```text
01,01,02,6F,00,30,FF,80,00,80,00,00,00
```

### 7.5 Comment Lines

Comment lines are allowed.

Recommended comment prefixes:

```text
#
//
;
```

Example:

```text
# sample record
01 01 02 6F 00 30 FF 80 00 80 00 00 00
```

### 7.6 Invalid Lines

Invalid lines must not stop the entire parsing operation unless parsing cannot continue at all.

Required behavior:

- Show an error or warning for the invalid line.
- Continue parsing subsequent lines when possible.
- Include invalid-line count in the parse summary.
- Provide enough information for the user to locate problematic lines.

---

## 8. Binary Log Parsing

### 8.1 Full Binary Log

A full binary log consists of:

```text
File Header
LogRecord[]
Optional Summary Block
```

If a File Header exists, the viewer must parse it before decoding records.

### 8.2 Summary Block Handling

If a Summary Block exists:

- Display Summary Block values as the primary summary information.
- Also recalculate summary values from decoded records.
- If recalculated values differ from Summary Block values, show a warning.

If a Summary Block does not exist:

- Show a warning that the Summary Block is missing.
- Display recalculated summary values instead.

Required warning example:

```text
Summary Block is missing. Displaying recalculated summary values from decoded records.
```

### 8.3 Incomplete or Unclean Logs

If the log appears incomplete, malformed, or not closed cleanly:

- Treat it as a warning unless decoding is impossible.
- Attempt to parse valid records that can be safely decoded.
- Clearly display parsing warnings.

---

## 9. Internal Data Model

After parsing, all inputs should be normalized to a common internal record model.

Recommended normalized record fields:

| Field | Description |
|---|---|
| `index` | Viewer-side record index |
| `sequence_number` | Sequence number if available |
| `rx_time_ms` | Child-side receive time if available |
| `elapsed_ms` | Elapsed time from first valid record |
| `rssi` | RSSI if available |
| `record_flags` | Decoded record flags if available |
| `crc_status` | CRC OK / CRC error / unknown |
| `application_data` | Raw 13-byte application data |
| `data_type` | Application data type |
| `status_flags` | Vehicle telemetry status flags |
| `speed_kmh` | Decoded vehicle speed in km/h |
| `rpm` | Decoded engine speed in rpm |
| `throttle_percent` | Decoded throttle percent or N/A |
| `temp1_c` | Decoded temperature 1 or N/A |
| `temp2_c` | Decoded temperature 2 or N/A |
| `raw_hex` | Raw application data hex string |
| `warnings` | Record-specific warnings |

---

## 10. Application Data Decoding

The viewer must decode Vehicle Telemetry V1 records.

### 10.1 Application Data Layout

`application_data` is exactly 13 bytes.

| Offset | Size | Field | Type | Unit / Encoding |
|---:|---:|---|---|---|
| 0 | 1 | `data_type` | `uint8_t` | `0x01` for Vehicle Telemetry V1 |
| 1 | 1 | `status_flags` | `uint8_t` | Bit field |
| 2 | 2 | `speed_x10` | `uint16_t` | km/h × 10 |
| 4 | 2 | `rpm_x100` | `uint16_t` | rpm / 100 |
| 6 | 1 | `throttle` | `uint8_t` | 0-255, or `0xFF` for N/A |
| 7 | 2 | `temp1_x10` | `int16_t` | °C × 10, or `0x8000` for N/A |
| 9 | 2 | `temp2_x10` | `int16_t` | °C × 10, or `0x8000` for N/A |
| 11 | 2 | `reserved` | `uint16_t` | Reserved, expected `0x0000` |

### 10.2 Byte Order

All multi-byte fields are big-endian.

### 10.3 Decoded Values

| Field | Decode Rule |
|---|---|
| `speed_kmh` | `speed_x10 / 10.0` |
| `rpm` | `rpm_x100 * 100` |
| `throttle_percent` | `throttle * 100 / 255`, unless `0xFF` |
| `temp1_c` | `temp1_x10 / 10.0`, unless `0x8000` |
| `temp2_c` | `temp2_x10 / 10.0`, unless `0x8000` |

### 10.4 Not Implemented Values

The viewer must display the following values as `N/A`:

| Field | Raw Value | Display |
|---|---:|---|
| `throttle` | `0xFF` | `N/A` |
| `temp1_x10` | `0x8000` | `N/A` |
| `temp2_x10` | `0x8000` | `N/A` |

These values must be excluded from min/max/average calculations.

### 10.5 Reserved Field

If `reserved != 0x0000`, the viewer should display a warning.

This warning is non-fatal.

---

## 11. Status Decoding

`status_flags` must be decoded as follows.

| Bit | Mask | Name | Description |
|---:|---:|---|---|
| 0 | `0x01` | `MEASURING` | Measurement is active |
| 1 | `0x02` | `PAUSED` | Measurement is paused |
| 2 | `0x04` | `SENSOR_ERROR` | Sensor error detected |
| 3-7 | - | Reserved | Expected 0 |

### 11.1 Measurement State Display

| MEASURING | PAUSED | Display |
|---:|---:|---|
| 0 | 0 | `Stopped` |
| 1 | 0 | `Measuring` |
| 0 | 1 | `Paused` |
| 1 | 1 | `Invalid` |

If reserved bits are non-zero, display a warning.

---

## 12. Summary View Requirements

The Summary view must contain two major areas:

1. Basic summary information area
2. Scrollable record list area

### 12.1 Basic Summary Information Area

The basic summary area must display:

- Input type detected
- Number of parsed records
- Number of valid records
- Number of invalid records
- Number of warning records
- Total detected lost records
- Whether File Header exists
- Whether Summary Block exists
- Summary source currently displayed
  - `Summary Block`
  - `Recalculated`
- File Header values if available
- Summary Block values if available
- Recalculated summary values
- Summary mismatch warning if applicable

### 12.2 Summary Source Priority

Summary display priority:

```text
1. If Summary Block exists and is valid, display Summary Block values as primary.
2. Always calculate summary values from parsed records.
3. If Summary Block is missing, display recalculated values as primary and show a warning.
4. If Summary Block differs from recalculated values, show a warning.
```

### 12.3 Summary Metrics

The viewer should calculate and/or display at least:

| Metric | Description |
|---|---|
| `min_speed_kmh` | Minimum valid speed |
| `max_speed_kmh` | Maximum valid speed |
| `avg_speed_kmh` | Average valid speed |
| `min_rpm` | Minimum valid RPM |
| `max_rpm` | Maximum valid RPM |
| `avg_rpm` | Average valid RPM |
| `min_temp1_c` | Minimum valid temperature 1, if available |
| `max_temp1_c` | Maximum valid temperature 1, if available |
| `min_temp2_c` | Minimum valid temperature 2, if available |
| `max_temp2_c` | Maximum valid temperature 2, if available |
| `min_rssi` | Minimum RSSI, if available |
| `max_rssi` | Maximum RSSI, if available |
| `packet_loss_count` | Total detected lost packets |
| `crc_error_count` | CRC error count |
| `duplicate_or_old_count` | Duplicate or old record count |

---

## 13. Record List / Table View Requirements

The viewer must provide a scrollable record list.

### 13.1 Required Columns

| Column | Description |
|---|---|
| `index` | Viewer-side row number |
| `sequence_number` | Sequence number |
| `rx_time_ms` | Receive time in ms, if available |
| `elapsed_time` | Time since first valid record |
| `speed_kmh` | Vehicle speed |
| `rpm` | Engine speed |
| `throttle` | Throttle opening, or `N/A` |
| `temp1` | Temperature 1, or `N/A` |
| `temp2` | Temperature 2, or `N/A` |
| `status` | `Stopped`, `Measuring`, `Paused`, or `Invalid` |
| `rssi` | RSSI, if available |
| `flags` | Decoded flags and warning status |
| `raw` | Raw application data hex |

### 13.2 Required Table Behaviors

The table must support:

- Scrollable record list.
- Row click selection.
- Selected row highlighting.
- Highlighting rows with packet loss before the record.
- Highlighting rows with CRC errors.
- Displaying unimplemented values as `N/A`.
- Toggle to hide/show Duplicate/Old records.
- Displaying warnings without stopping the entire viewer.

### 13.3 Linked Selection with Graph

Required behavior:

| Action | Required Result |
|---|---|
| Click table row | Highlight corresponding graph point |
| Click graph point | Select corresponding table row and scroll it into view |
| Hover table row | Optional graph hover highlight |
| Hover graph point | Optional row hover highlight |

---

## 14. Graph View Requirements

The Graph view must plot decoded log data in time-series order.

### 14.1 Required Graphs / Metrics

The viewer must support graphing at least:

| Graph Metric | Y-axis |
|---|---|
| Speed | km/h |
| RPM | rpm |
| RSSI | dBm |
| Temperature 1 | °C |
| Temperature 2 | °C |
| Throttle | % |
| Packet loss events | lost count or event marker |

### 14.2 X-Axis Options

The viewer must support the following X-axis options:

| X-axis | Description |
|---|---|
| `elapsed_time` | Time elapsed from the first valid record |
| `record_index` | Viewer-side record index |
| `sequence_number` | Transport sequence number |
| `rx_time_ms` | Child-side receive timestamp |
| `tx_millis` | Parent-side transmit timestamp, if available |

Default X-axis:

```text
elapsed_time
```

### 14.3 Graph Selection and Highlight

The graph must support:

- Highlight selected record from Summary/Table view.
- Click graph point to select corresponding table row.
- Tooltip display on hover.
- Packet loss marker display.
- CRC error marker display if such records are shown.
- Optional range selection.

### 14.4 Tooltip Fields

Graph tooltip should display:

- index
- sequence number
- elapsed time
- speed
- rpm
- throttle
- temperature 1
- temperature 2
- RSSI
- status
- packet loss before this record
- raw application data hex

---

## 15. Selection and Highlight Behavior

### 15.1 Single Selection

Single record selection must be shared between:

- Record table
- Graph
- Detail panel, if implemented

### 15.2 Range Selection

The viewer should support range selection.

If range selection is implemented, the viewer should display range summary values:

- selected record count
- min/max/average speed
- min/max/average rpm
- min/max/average RSSI if available
- packet loss count inside range

### 15.3 Highlighting Rules

| Condition | Required UI Treatment |
|---|---|
| Selected record | Strong highlight |
| Packet loss before record | Warning highlight |
| CRC error | Error highlight |
| Duplicate/Old record | Muted or special highlight |
| Invalid application data | Warning or error highlight |
| Missing/unimplemented value | Show as `N/A` |

---

## 16. Error and Warning Handling

The viewer should continue processing whenever possible.

Only errors that make parsing impossible should be fatal.

### 16.1 Warning-First Policy

Detected errors should be treated as warnings unless they are fatal.

Examples of non-fatal warnings:

- Invalid text line
- CRC error in a record
- Missing Summary Block
- Summary mismatch
- Record count mismatch
- Reserved field is non-zero
- Unsupported application data record mixed into otherwise valid data
- Duplicate or old sequence number
- Packet loss detected

### 16.2 Fatal Errors

Fatal errors include:

- Empty input
- No decodable records found
- Unsupported binary format with no recoverable records
- File Header indicates an unsupported format version and no safe fallback exists

### 16.3 Required Error Display

The viewer must display:

- Error summary
- Warning summary
- Invalid line count for text input
- Invalid record count
- Record-level warnings in the table

---

## 17. Specific Error / Warning Detection

The viewer should detect the following conditions.

| Condition | Severity | Required Handling |
|---|---|---|
| Header magic mismatch | Fatal or warning depending on fallback | Try fallback parsing if possible |
| Unsupported format version | Fatal or warning depending on fallback | Show unsupported version warning |
| Record size mismatch | Warning | Parse recoverable records if possible |
| Summary Block missing | Warning | Use recalculated summary |
| Summary mismatch | Warning | Display both values or mismatch notice |
| CRC8 mismatch | Warning | Mark record corrupted |
| Transport CRC16 mismatch | Warning | Mark record corrupted if available |
| Sequence number gap | Warning | Mark packet loss |
| Sequence duplicate/old | Warning | Mark duplicate/old |
| Reserved field non-zero | Warning | Mark record warning |
| Unimplemented value | Info | Display `N/A` |
| Invalid text line | Warning | Continue parsing next line |

---

## 18. Export Requirements

### 18.1 Required Export

CSV export is required.

The viewer must be able to export decoded records as CSV.

### 18.2 Recommended CSV Columns

```text
index,sequence_number,rx_time_ms,elapsed_ms,rssi,status,speed_kmh,rpm,throttle_percent,temp1_c,temp2_c,flags,app_data_hex,warnings
```

### 18.3 CSV Export Rules

- Use one row per decoded record.
- Use `N/A` or empty field for unavailable values.
- Include warning information per row.
- Preserve raw application data hex.
- Export currently filtered data or all data depending on user selection.

The UI should clearly indicate whether the export target is:

- all records
- filtered records
- selected range

### 18.4 Optional Future Exports

The following export formats are optional but should be documented for future implementation:

| Format | Purpose |
|---|---|
| JSON | Tool integration and structured analysis |
| Decoded text | Human-readable debug sharing |
| Graph image | Report generation |
| Filtered binary | Extracting a subset of valid records |

---

## 19. UI Layout Requirements

The viewer should follow this layout concept:

```text
+--------------------------------------------------+
| Header: File open / Paste text / Export / Status |
+----------------------+---------------------------+
| Summary / Table      | Graph                     |
| - file info          | - metric selector         |
| - summary block      | - time-series chart       |
| - recalculated stats | - selected point marker   |
| - record list        |                           |
+----------------------+---------------------------+
| Raw / Decode detail panel                        |
+--------------------------------------------------+
```

### 19.1 Header Area

The header area should include:

- File open button
- Text paste input access
- Parse / Load button if needed
- Export CSV button
- Input status
- Warning/error count

### 19.2 Summary / Table Area

The Summary / Table area should include:

- Basic file/log information
- Summary Block values if present
- Recalculated summary values
- Warning when Summary Block is absent
- Warning when Summary Block and recalculated values differ
- Scrollable record table

### 19.3 Graph Area

The Graph area should include:

- Metric selector
- X-axis selector
- Time-series graph
- Selected point highlight
- Packet loss markers
- CRC error markers where applicable

### 19.4 Raw / Detail Panel

A detail panel should show details for the selected record:

- Full decoded record
- Raw bytes
- Application data fields
- Flags
- Warnings
- CRC status

---

## 20. Filtering and Display Toggles

The viewer should support the following toggles:

| Toggle | Required |
|---|---:|
| Hide Duplicate/Old records | Yes |
| Show CRC error records | Yes |
| Show packet loss markers | Yes |
| Show `N/A` values | Yes |
| Show raw hex | Yes |
| Show warnings only | Recommended |
| Show selected range only | Recommended |

Default behavior:

- Duplicate/Old records may be visible by default during development.
- A toggle must allow hiding Duplicate/Old records.
- CRC error records should be visible but clearly marked.

---

## 21. Performance Requirements

The viewer should remain usable for large logs.

Recommended performance considerations:

- Large record tables should use efficient scrolling or virtualized rendering.
- Parsing should report progress for very large files if needed.
- Graph rendering should remain responsive.
- The viewer should avoid blocking the UI for long parsing operations where possible.

No fixed maximum file size is defined in this specification.

---

## 22. Version Compatibility

The viewer must check and display:

- Log format version
- Transport protocol version if available
- Application data type

### 22.1 Unknown Application Data Type

If an unknown `data_type` is encountered:

- Do not crash.
- Show raw data.
- Mark the record as unsupported.
- Continue parsing subsequent records.

### 22.2 Future Reserved Fields

If reserved fields or reserved bits are non-zero:

- Show a warning.
- Continue parsing known fields.

---

## 23. Test Data Requirements

The viewer implementation should be tested with at least the following cases:

### 23.1 Valid Inputs

- Full binary log with Summary Block
- Full binary log without Summary Block
- LogRecord V1 sequence
- Raw appData[13] sequence
- Serial log paste containing `Payload:` lines
- Whitespace-separated hex text
- Comma-separated hex text
- `0x` prefixed hex text
- Text containing comments

### 23.2 Error / Warning Inputs

- Invalid line mixed with valid text records
- CRC error record
- Missing Summary Block
- Summary mismatch
- Sequence gap
- Duplicate sequence
- Reserved field non-zero
- Unknown application data type
- Incomplete final record
- Empty input

---

## 24. Recommended User Workflow

Primary workflow:

```text
1. User copies ESP32 serial monitor output.
2. User pastes text into the viewer TextArea.
3. Viewer auto-detects serial log format.
4. Viewer extracts payload records.
5. Viewer displays summary and warnings.
6. User inspects table and graph.
7. User clicks a table row or graph point to inspect a record.
8. User exports decoded CSV if needed.
```

Secondary workflow:

```text
1. User selects a .bin log file.
2. Viewer parses File Header, records, and optional Summary Block.
3. Viewer displays Summary Block as primary summary if valid.
4. Viewer also recalculates summary values.
5. Viewer warns if Summary Block is missing or inconsistent.
6. User inspects graph and exports CSV if needed.
```

---

## 25. Current Decisions

| Topic | Decision |
|---|---|
| Primary input workflow | ESP32 serial log paste |
| Binary input | Supported |
| TextArea paste input | Required |
| Whitespace-separated hex | Allowed |
| Newline-separated records | Allowed |
| `0x` prefix | Allowed |
| Comma-separated values | Allowed |
| Comment lines | Allowed |
| Invalid lines | Warn and continue |
| Input auto-detection | Use specified detection order |
| Summary Block priority | Prefer Summary Block when present |
| Recalculated summary | Always compute |
| Missing Summary Block | Warn and use recalculated summary |
| Table columns | Use proposed full column set |
| Table-to-graph selection | Required |
| Graph-to-table selection | Required |
| Packet loss highlighting | Required |
| CRC error display | Required |
| Duplicate/Old toggle | Required |
| N/A display | Required |
| Graph features | Proposed features required |
| CSV export | Required |
| Other exports | Document only |
| Non-fatal errors | Warning, continue processing |
| Server upload | Not allowed |
| Implementation technology | Not specified |

