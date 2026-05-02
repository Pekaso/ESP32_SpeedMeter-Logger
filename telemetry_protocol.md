# Vehicle Telemetry Application Data / Log Format Specification

## 1. Purpose

This document defines the binary data format used between the parent unit and child unit in the vehicle telemetry system.

The purpose of this specification is to keep implementations consistent across:

- Parent unit firmware
- Child unit firmware
- Log writer
- Log parser
- Future tools generated or modified by humans or LLMs

The parent unit sends vehicle telemetry data to the child unit via an E220-900T22S(JP) LoRa module.  
The LoRa payload contains a common transport header and a 13-byte application data field.

This document mainly defines the **13-byte application data format** and the **recommended binary log format** used by the child unit.

---

## 2. Byte Order

All multi-byte integer fields in this specification use:

```text
Big-endian / network byte order
```

Example:

```text
uint16_t value = 0x1234

byte[0] = 0x12
byte[1] = 0x34
```

Do not directly transmit or store C/C++ structs without explicit packing/unpacking functions.

---

## 3. Numeric Types

| Type | Size | Description |
|---|---:|---|
| `uint8_t` | 1 byte | Unsigned 8-bit integer |
| `int8_t` | 1 byte | Signed 8-bit integer |
| `uint16_t` | 2 bytes | Unsigned 16-bit integer, big-endian |
| `int16_t` | 2 bytes | Signed 16-bit integer, big-endian |
| `uint32_t` | 4 bytes | Unsigned 32-bit integer, big-endian |

---

## 4. LoRa Transport Payload Overview

The current LoRa transport payload length is:

```text
29 bytes
```

This is selected because the E220-900T22S(JP) is used in Fixed-block mode with:

```text
32-byte subpacket
```

In Fixed-block mode, the UART frame sent to the module includes:

```text
2 bytes target address
1 byte target channel
N bytes application payload
```

Therefore:

```text
32-byte subpacket - 3-byte destination header = 29-byte effective payload
```

The 29-byte transport payload is structured as follows:

| Offset | Size | Field | Description |
|---:|---:|---|---|
| 0 | 1 | `magic` | Fixed value `0xE2` |
| 1 | 1 | `protocol_version` | Current version: `0x01` |
| 2 | 2 | `sender_node_id` | Parent node ID |
| 4 | 4 | `sequence_number` | Incremented by 1 for each transmitted frame |
| 8 | 4 | `tx_millis` | Parent-side `millis()` value at transmission |
| 12 | 2 | `app_counter` | Application counter |
| 14 | 13 | `application_data` | Vehicle telemetry data, defined below |
| 27 | 2 | `crc16` | CRC16-CCITT-FALSE over bytes `0..26` |

---

## 5. Transport Payload Constants

| Name | Value | Description |
|---|---:|---|
| `TRANSPORT_PAYLOAD_SIZE` | `29` | Total LoRa payload size |
| `APPLICATION_DATA_SIZE` | `13` | Application data size |
| `MAGIC` | `0xE2` | Transport magic byte |
| `PROTOCOL_VERSION` | `0x01` | Current protocol version |

---

## 6. CRC16

The transport payload uses:

```text
CRC16-CCITT-FALSE
```

Parameters:

| Parameter | Value |
|---|---|
| Polynomial | `0x1021` |
| Initial value | `0xFFFF` |
| RefIn | `false` |
| RefOut | `false` |
| XorOut | `0x0000` |

CRC calculation range:

```text
transport_payload[0] through transport_payload[26]
```

CRC storage:

```text
transport_payload[27] = CRC high byte
transport_payload[28] = CRC low byte
```

---

# 7. Application Data Format

## 7.1 Overview

The `application_data` field is exactly:

```text
13 bytes
```

It contains the main vehicle telemetry values sent from the parent unit to the child unit.

Current requirements:

- Vehicle speed
- Engine RPM, represented as `x100 rpm`
- Throttle opening, reserved for future implementation
- Temperature 1, reserved for future implementation
- Temperature 2, reserved for future implementation
- Measurement start/stop status

---

## 7.2 Application Data Layout

| Offset | Size | Field | Type | Unit / Encoding | Description |
|---:|---:|---|---|---|---|
| 0 | 1 | `data_type` | `uint8_t` | Enum | Application data type |
| 1 | 1 | `status_flags` | `uint8_t` | Bit field | Measurement state and error flags |
| 2 | 2 | `speed_x10` | `uint16_t` | km/h × 10 | Vehicle speed |
| 4 | 2 | `rpm_x100` | `uint16_t` | rpm / 100 | Engine speed |
| 6 | 1 | `throttle` | `uint8_t` | 0-255 | Throttle opening |
| 7 | 2 | `temp1_x10` | `int16_t` | °C × 10 | Temperature sensor 1 |
| 9 | 2 | `temp2_x10` | `int16_t` | °C × 10 | Temperature sensor 2 |
| 11 | 2 | `reserved` | `uint16_t` | Reserved | Must be `0x0000` when unused |

Total:

```text
13 bytes
```

---

## 7.3 `data_type`

| Value | Name | Description |
|---:|---|---|
| `0x01` | `VEHICLE_TELEMETRY` | Standard vehicle telemetry data |
| `0x02` - `0x7F` | Reserved | Reserved for future normal data types |
| `0x80` - `0xFE` | Reserved | Reserved for future control/debug/system data types |
| `0xFF` | Invalid | Must not be used for valid records |

Current implementation must set:

```text
data_type = 0x01
```

---

## 7.4 `status_flags`

`status_flags` is a bit field.

| Bit | Mask | Name | Description |
|---:|---:|---|---|
| 0 | `0x01` | `MEASURING` | Measurement is active |
| 1 | `0x02` | `PAUSED` | Measurement is paused |
| 2 | `0x04` | `SENSOR_ERROR` | One or more sensor values may be invalid |
| 3 | `0x08` | Reserved | Must be 0 |
| 4 | `0x10` | Reserved | Must be 0 |
| 5 | `0x20` | Reserved | Must be 0 |
| 6 | `0x40` | Reserved | Must be 0 |
| 7 | `0x80` | Reserved | Must be 0 |

### Measurement State Encoding

Recommended usage:

| State | `MEASURING` | `PAUSED` | Meaning |
|---|---:|---:|---|
| Stopped | 0 | 0 | Measurement stopped |
| Measuring | 1 | 0 | Measurement active |
| Paused | 0 | 1 | Measurement paused |
| Invalid / reserved | 1 | 1 | Should not be used |

---

## 7.5 `speed_x10`

Vehicle speed.

```text
speed_x10 = speed_kmh × 10
```

Examples:

| Speed | Encoded value |
|---:|---:|
| 0.0 km/h | `0` |
| 12.3 km/h | `123` |
| 60.0 km/h | `600` |
| 123.4 km/h | `1234` |

Range:

```text
0.0 to 6553.5 km/h
```

For normal vehicle use, values outside realistic range should be treated as invalid by the application layer.

---

## 7.6 `rpm_x100`

Engine speed.

```text
rpm_x100 = engine_rpm / 100
```

Examples:

| Engine RPM | Encoded value |
|---:|---:|
| 0 rpm | `0` |
| 800 rpm | `8` |
| 4800 rpm | `48` |
| 12000 rpm | `120` |

Range:

```text
0 to 6,553,500 rpm
```

For normal vehicle use, values outside realistic range should be treated as invalid by the application layer.

---

## 7.7 `throttle`

Throttle opening.

```text
0x00 = fully closed
0xFF = not implemented / invalid
```

When implemented:

| Value | Meaning |
|---:|---|
| `0` | 0% |
| `255` | 100% |

Recommended conversion:

```text
throttle_percent = throttle × 100 / 255
```

Current implementation:

```text
throttle = 0xFF
```

---

## 7.8 `temp1_x10` and `temp2_x10`

Temperature values.

```text
temp_x10 = temperature_celsius × 10
```

Examples:

| Temperature | Encoded value |
|---:|---:|
| -10.0 °C | `-100` |
| 0.0 °C | `0` |
| 25.3 °C | `253` |
| 100.0 °C | `1000` |

Not implemented / invalid value:

```text
0x8000
```

Interpreted as signed 16-bit:

```text
INT16_MIN = -32768
```

Current implementation:

```text
temp1_x10 = 0x8000
temp2_x10 = 0x8000
```

Parsers must exclude `0x8000` from min/max summary calculations.

---

## 7.9 `reserved`

Reserved for future use.

Current implementation must set:

```text
reserved = 0x0000
```

Receivers and log parsers must ignore this field unless a future protocol version defines it.

---

# 8. Recommended Application Data Pack Function

```cpp
static const uint8_t APP_DATA_TYPE_VEHICLE = 0x01;

static const uint8_t STATUS_MEASURING    = 0x01;
static const uint8_t STATUS_PAUSED       = 0x02;
static const uint8_t STATUS_SENSOR_ERROR = 0x04;

static const uint8_t THROTTLE_NOT_IMPL = 0xFF;
static const int16_t TEMP_NOT_IMPL = INT16_MIN;

static void put_u16_be(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v >> 8);
  p[1] = (uint8_t)(v & 0xFF);
}

static void put_i16_be(uint8_t *p, int16_t v) {
  uint16_t u = (uint16_t)v;
  p[0] = (uint8_t)(u >> 8);
  p[1] = (uint8_t)(u & 0xFF);
}

static void pack_vehicle_app_data(
  uint8_t appData[13],
  uint16_t speed_x10,
  uint16_t rpm_x100,
  bool measuring
) {
  memset(appData, 0, 13);

  appData[0] = APP_DATA_TYPE_VEHICLE;

  uint8_t status = 0;
  if (measuring) {
    status |= STATUS_MEASURING;
  }
  appData[1] = status;

  put_u16_be(&appData[2], speed_x10);
  put_u16_be(&appData[4], rpm_x100);

  appData[6] = THROTTLE_NOT_IMPL;

  put_i16_be(&appData[7], TEMP_NOT_IMPL);
  put_i16_be(&appData[9], TEMP_NOT_IMPL);

  put_u16_be(&appData[11], 0x0000);
}
```

---

# 9. Binary Log Format

## 9.1 Design Policy

The log file should not be a raw sequence of `appData[13]` only.

Instead, the recommended format is:

```text
File Header
Log Record[]
Summary Block
```

Reasons:

- The file can be identified by magic bytes.
- Format version can be checked.
- Record size can be changed in future versions.
- Sequence number and timestamp can be preserved.
- RSSI and packet loss information can be stored.
- Min/max values can be stored separately from time-series records.

Min/max records should not be mixed into the normal telemetry record stream using `data_type`.

---

## 9.2 File Structure

```text
+----------------+
| File Header    |
+----------------+
| Log Record 0   |
+----------------+
| Log Record 1   |
+----------------+
| Log Record 2   |
+----------------+
| ...            |
+----------------+
| Summary Block  |
+----------------+
```

---

## 9.3 File Header V1

Recommended size:

```text
32 bytes
```

| Offset | Size | Field | Type | Description |
|---:|---:|---|---|---|
| 0 | 4 | `magic` | bytes | ASCII `"SCUB"` |
| 4 | 1 | `format_version` | `uint8_t` | Current version: `0x01` |
| 5 | 1 | `record_size` | `uint8_t` | Log record size in bytes |
| 6 | 2 | `sample_period_ms` | `uint16_t` | Nominal sample period in ms |
| 8 | 4 | `start_time_unix` | `uint32_t` | Unix time, or 0 if unavailable |
| 12 | 4 | `record_count` | `uint32_t` | Number of records, or `0xFFFFFFFF` while writing |
| 16 | 4 | `lost_record_count` | `uint32_t` | Total detected lost records, or 0 while writing |
| 20 | 4 | `flags` | `uint32_t` | File-level flags |
| 24 | 8 | `reserved` | bytes | Must be zero |

Current values:

```text
magic            = "SCUB"
format_version   = 0x01
record_size      = 24
sample_period_ms = 1000
```

---

## 9.4 File Header Flags

| Bit | Mask | Name | Description |
|---:|---:|---|---|
| 0 | `0x00000001` | `HAS_SUMMARY` | Summary block exists |
| 1 | `0x00000002` | `TIME_VALID` | `start_time_unix` is valid |
| 2 | `0x00000004` | `CLOSED_CLEANLY` | File was closed normally |
| 3-31 | - | Reserved | Must be 0 |

---

# 10. Log Record V1

## 10.1 Overview

Recommended log record size:

```text
24 bytes
```

A log record represents one received telemetry frame.

It includes the received application data plus metadata observed by the child unit.

---

## 10.2 Log Record Layout

| Offset | Size | Field | Type | Description |
|---:|---:|---|---|---|
| 0 | 4 | `sequence_number` | `uint32_t` | Sequence number from transport payload |
| 4 | 4 | `rx_time_ms` | `uint32_t` | Child-side `millis()` at receive time |
| 8 | 1 | `rssi` | `int8_t` | RSSI value in dBm |
| 9 | 1 | `record_flags` | `uint8_t` | Record status flags |
| 10 | 13 | `application_data` | bytes | Application data defined in section 7 |
| 23 | 1 | `record_crc8` | `uint8_t` | CRC8 over bytes `0..22` |

Total:

```text
24 bytes
```

---

## 10.3 Record Flags

| Bit | Mask | Name | Description |
|---:|---:|---|---|
| 0 | `0x01` | `CRC_OK` | Transport CRC was valid |
| 1 | `0x02` | `LOSS_BEFORE_THIS` | One or more sequence numbers were skipped before this record |
| 2 | `0x04` | `DUPLICATE_OR_OLD` | Received sequence number was duplicate or older |
| 3 | `0x08` | `APP_DATA_VALID` | Application data type and values were accepted |
| 4-7 | - | Reserved | Must be 0 |

---

## 10.4 Record CRC8

The record CRC8 is used to detect corruption in stored log data.

Recommended algorithm:

```text
CRC-8/ATM
```

Parameters:

| Parameter | Value |
|---|---|
| Polynomial | `0x07` |
| Initial value | `0x00` |
| RefIn | `false` |
| RefOut | `false` |
| XorOut | `0x00` |

CRC calculation range:

```text
log_record[0] through log_record[22]
```

CRC storage:

```text
log_record[23]
```

---

# 11. Summary Block V1

## 11.1 Design Policy

Min/max values should be stored in the Summary Block, not as special `data_type` records inside the normal telemetry stream.

The normal record stream should remain a pure time-series log.

This makes later processing easier for:

- CSV conversion
- Graph generation
- Packet loss analysis
- Max/min recalculation
- Partial log recovery

---

## 11.2 Summary Block Layout

Recommended size:

```text
32 bytes
```

| Offset | Size | Field | Type | Description |
|---:|---:|---|---|---|
| 0 | 4 | `summary_magic` | bytes | ASCII `"SUMM"` |
| 4 | 2 | `min_speed_x10` | `uint16_t` | Minimum valid speed |
| 6 | 2 | `max_speed_x10` | `uint16_t` | Maximum valid speed |
| 8 | 2 | `min_rpm_x100` | `uint16_t` | Minimum valid RPM |
| 10 | 2 | `max_rpm_x100` | `uint16_t` | Maximum valid RPM |
| 12 | 2 | `min_temp1_x10` | `int16_t` | Minimum valid temp1 |
| 14 | 2 | `max_temp1_x10` | `int16_t` | Maximum valid temp1 |
| 16 | 2 | `min_temp2_x10` | `int16_t` | Minimum valid temp2 |
| 18 | 2 | `max_temp2_x10` | `int16_t` | Maximum valid temp2 |
| 20 | 4 | `total_records` | `uint32_t` | Number of valid records |
| 24 | 4 | `lost_records` | `uint32_t` | Number of detected lost records |
| 28 | 4 | `crc32` | `uint32_t` | CRC32 of the log body or summary |

---

## 11.3 Handling Not Implemented Values

The following values must be excluded from min/max calculations:

| Field | Invalid value |
|---|---:|
| `throttle` | `0xFF` |
| `temp1_x10` | `0x8000` |
| `temp2_x10` | `0x8000` |

If no valid value exists for a min/max field, use the invalid value for both min and max.

Example:

```text
min_temp1_x10 = 0x8000
max_temp1_x10 = 0x8000
```

---

# 12. Packet Loss Detection

The parent unit increments `sequence_number` by 1 for each transmitted frame.

The child unit detects packet loss as follows:

```text
expected_sequence = previous_sequence + 1
```

If:

```text
received_sequence > expected_sequence
```

Then:

```text
lost_count = received_sequence - expected_sequence
```

The child unit should:

- Add `lost_count` to total lost count.
- Set `LOSS_BEFORE_THIS` in the current record flags.
- Store the received record normally.

If:

```text
received_sequence <= previous_sequence
```

Then the record is duplicate or old.

The child unit should:

- Set `DUPLICATE_OR_OLD`.
- Decide whether to store or discard based on implementation policy.

Recommended policy:

```text
Store duplicate/old records only in debug builds.
Discard them in normal operation.
```

---

# 13. Recommended Parser Behavior

A parser should:

1. Read and validate File Header.
2. Confirm `magic == "SCUB"`.
3. Confirm supported `format_version`.
4. Use `record_size` to iterate records.
5. Stop before Summary Block if `"SUMM"` is detected.
6. Validate each `record_crc8`.
7. Decode `application_data`.
8. Ignore reserved fields.
9. Exclude invalid/unimplemented values from min/max calculation.
10. Prefer recalculated summary values if stored Summary Block is missing or invalid.

---

# 14. Recommended Implementation Rules

## 14.1 Do

- Use explicit pack/unpack functions.
- Use big-endian for all multi-byte fields.
- Keep `application_data` exactly 13 bytes.
- Keep normal telemetry records and summary data separate.
- Validate `magic`, `protocol_version`, `data_type`, and CRC.
- Treat reserved fields as zero when transmitting.
- Ignore reserved fields when receiving.

## 14.2 Do Not

- Do not send raw C/C++ structs directly.
- Do not depend on compiler struct packing.
- Do not mix min/max summary data into the normal telemetry record stream.
- Do not use reserved bits for new meanings without updating `format_version`.
- Do not treat invalid temperature value `0x8000` as real temperature data.

---

# 15. Versioning Policy

## 15.1 Transport Protocol Version

`protocol_version` in the transport payload must be incremented if the transport payload layout changes.

Current:

```text
protocol_version = 0x01
```

## 15.2 Application Data Type

`data_type` identifies the application data layout.

Current:

```text
data_type = 0x01
```

If the 13-byte vehicle telemetry format changes incompatibly, define a new `data_type`.

Example:

```text
0x01 = Vehicle telemetry V1
0x02 = Vehicle telemetry V2
```

## 15.3 Log Format Version

`format_version` in the file header must be incremented if the log file layout changes incompatibly.

Current:

```text
format_version = 0x01
```

---

# 16. Current Version Summary

## Transport Payload V1

```text
Total size: 29 bytes
Application data size: 13 bytes
CRC: CRC16-CCITT-FALSE
```

## Application Data V1

```text
data_type: 0x01
size: 13 bytes
fields:
- status flags
- speed_x10
- rpm_x100
- throttle
- temp1_x10
- temp2_x10
- reserved
```

## Log Format V1

```text
Header: 32 bytes
Record: 24 bytes
Summary: 32 bytes
```

---

# 17. Example Application Data

Example:

```text
Vehicle speed: 62.3 km/h
Engine speed: 4800 rpm
Measuring: true
Throttle: not implemented
Temperature 1: not implemented
Temperature 2: not implemented
Reserved: 0x0000
```

Encoded:

| Offset | Value | Description |
|---:|---:|---|
| 0 | `0x01` | Vehicle telemetry |
| 1 | `0x01` | Measuring |
| 2 | `0x02` | speed high byte |
| 3 | `0x6F` | speed low byte, 623 |
| 4 | `0x00` | rpm high byte |
| 5 | `0x30` | rpm low byte, 48 |
| 6 | `0xFF` | throttle not implemented |
| 7 | `0x80` | temp1 high byte |
| 8 | `0x00` | temp1 low byte |
| 9 | `0x80` | temp2 high byte |
| 10 | `0x00` | temp2 low byte |
| 11 | `0x00` | reserved high byte |
| 12 | `0x00` | reserved low byte |

Hex dump:

```text
01 01 02 6F 00 30 FF 80 00 80 00 00 00
```
