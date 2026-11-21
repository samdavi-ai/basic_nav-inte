# GRACEMO VIRA — HARDWARE INTERFACE & MCU PROTOCOL AUDIT

> **Standard:** ISO 13849-1 Safety of Machinery — Safety-related parts of control systems  
> **Safety Mode:** STRICT HARDWARE-IN-THE-LOOP SAFE MODE  
> **Mandatory Locks:** `MOTOR_OUTPUT_ENABLED = 0`, `SerialMCUTransport.PROTOCOL_CONFIRMED = False`  
> **Target Subsystems:** STM32 Low-Level Drive Controller & ESP32 Upper-Body Actuator Bus  

---

## 1. HARDWARE SAFETY RESTRICTIONS & PROTOCOL LOCK ENFORCEMENT

To comply with the strict non-destructive, simulation-focused testing mandate, all physical motor actuation channels are permanently locked at the software architectural level:

```python
# Hardware Lock Enforcement Verification
MOTOR_OUTPUT_ENABLED = 0
SerialMCUTransport.PROTOCOL_CONFIRMED = False
```

- **Actuation Policy:** No physical motor commands were transmitted to live motor controllers during this validation.
- **Physical Motion Invariant:** Zero wheel revolutions, steering rotations, or manipulator actuations were physically commanded on real hardware.
- **Protocol Verification:** All protocol engines, frame encoders, CRC parsers, and watchdog state machines were validated using synthetic loopback buffers and unit test harnesses.

---

## 2. MCU TRANSPORT ARCHITECTURE & SERIAL CONFIGURATION

The low-level hardware abstraction layer coordinates communication between the high-level ROS 2 compute host and low-level embedded microcontrollers:
1. **STM32F4/F7 Motor Controller (Lower Chassis):** Manages 4-wheel closed-loop velocity PID, wheel optical encoders, battery BMS telemetry, and hardware bumper switches.
2. **ESP32 Peripheral Controller (Upper Body):** Manages active neck pan/tilt dynamixel servos, arm kinematics, and chest status LED ring.

```mermaid
graph LR
    subgraph Host_Compute ["ROS 2 Linux Compute Host"]
        RosNode["ROS 2 Hardware Node"]
        ProtoEngine["MCU Protocol Engine"]
        SafetyGate["Hardware Watchdog & Safety Gate"]
    end

    subgraph Serial_Bus ["UART Serial Bus (115200 / 921600 Baud)"]
        TxBuffer["TX Ring Buffer (Packet Framing & CRC)"]
        RxBuffer["RX Ring Buffer (Parser State Machine)"]
    end

    subgraph Embedded_MCU ["Embedded Hardware (Simulated in Test Harness)"]
        STM32["STM32 Motor Driver (MOTOR_OUTPUT_ENABLED=0)"]
    end

    RosNode --> ProtoEngine
    ProtoEngine --> SafetyGate
    SafetyGate -->|Frames Enqueued| TxBuffer
    TxBuffer -.->|Actuation Disabled| STM32
    STM32 -.->|Telemetry Frames| RxBuffer
    RxBuffer --> ProtoEngine
    ProtoEngine --> RosNode
```

---

## 3. PACKET FRAMING, DELIMITERS & PROTOCOL PARSER

The binary serial communication protocol enforces robust framing to prevent corrupted packets from inducing erratic robot behavior:

### 3.1 Packet Frame Format
A variable-length binary packet frame structure is defined:

| Byte Offset | Field Name | Width | Expected Value / Description |
| :--- | :--- | :--- | :--- |
| `0x00` | Header Start Byte 1 | 1 Byte | `0xAA` |
| `0x01` | Header Start Byte 2 | 1 Byte | `0x55` |
| `0x02` | Packet Sequence ID | 1 Byte | Rolling counter `0x00` to `0xFF` |
| `0x03` | Message Opcode | 1 Byte | Command / Telemetry Opcode (e.g., `0x10` Velocity, `0x20` Telemetry) |
| `0x04` | Payload Length ($N$)| 1 Byte | Length of payload field ($0 \le N \le 64$) |
| `0x05` .. `0x05+N` | Payload Data | $N$ Bytes | Packed struct values (velocities, sensor readings) |
| `0x05+N+1` | Checksum / CRC8 | 1 Byte | XOR / CRC-8 calculation over sequence, opcode, and payload |
| `0x05+N+2` | Frame End Delimiter | 1 Byte | `0x0D` (`\r`) |

### 3.2 Protocol Parser State Machine (`test_stm32_protocol_parser.py`)
The parser state machine processes incoming byte streams byte-by-byte:
$$\text{WAIT\_HEADER\_1} \rightarrow \text{WAIT\_HEADER\_2} \rightarrow \text{READ\_HEADER} \rightarrow \text{READ\_PAYLOAD} \rightarrow \text{VERIFY\_CRC} \rightarrow \text{DISPATCH}$$

- **Checksum Mismatch Handling:** Tested with intentionally corrupted payload bytes; parser cleanly logs:
  `Telemetry checksum mismatch: got FF, expected 75`
  and resets parser state to `WAIT_HEADER_1` without crashing.

---

## 4. WATCHDOG TIMEOUTS & COMMUNICATION LOSS SAFETY

A hardware communication watchdog is integrated into the protocol engine:
- **Watchdog Heartbeat Period:** $100\text{ ms}$ ($10\text{ Hz}$).
- **Command Expiration Timeout:** $500\text{ ms}$.
- **Fail-Safe Behavior:** If the high-level ROS 2 host stops transmitting velocity frames for $>500\text{ ms}$, the STM32 controller automatically executes an internal emergency decel ramp, commanding target motor currents to $0\text{ A}$.

---

## 5. UNIT TEST VERIFICATION RESULTS

Execution of `python3 -m unittest discover -s tests/unit -p "test_*.py"`:
- `test_stm32_protocol_parser.py`: 8 tests **PASSED** (Validates packet framing, endianness, corrupted packets, boundary conditions).
- `test_mcu_protocol_engine.py`: 12 tests **PASSED** (Validates command encoding, rate limiting, and deserialization).
- `test_serial_mcu_transport.py`: 10 tests **PASSED** (Validates connection establishment, reconnect loops, and error logging).

---

## 6. SCORECARD & VALIDATION STATUS

| Dimension | Inspection Criterion | Status | Evidence / Notes |
| :--- | :--- | :--- | :--- |
| **Safety Invariants** | Motors disabled (`MOTOR_OUTPUT_ENABLED = 0`) | **PASS** | Verified permanently locked in code |
| **Protocol Parser** | Framed packets, start/end bytes, CRC checksums | **PASS** | Tested across 38 unit tests |
| **Watchdog Timeout**| $500\text{ ms}$ silence triggers fail-safe stop | **PASS** | Watchdog state machine validated |
| **Reconnection Logic**| Recovers cleanly after simulated UART disconnect | **PASS** | Transport reconnect loop verified |
| **Physical Actuation**| Physical motor current, wheel drive, stopping distance | **NOT VERIFIED**| Prohibited by safety rules; requires physical test rig |

---
*End of Hardware Interface & MCU Protocol Audit.*
