# Automotive Telematics Control Unit (C++17)

A compact telematics stack whose only external dependency is OpenSSL: reads vehicle data over CAN (OBD-II),
fuses GNSS position, detects driving and vehicle-health events, segments trips,
and uploads everything over MQTT or HTTPS (TLS 1.2/1.3, mutual TLS) through a
store-and-forward link that survives loss of cellular coverage.

```
 GNSS (NMEA) ──► gps::parseNmea ─┐
                                 ├─► VehicleState ─► EventDetector ─┐
 CAN bus ──► OBD-II Mode 01 ─────┘                └► TripTracker ───┤
 (SocketCAN or simulator)                                           ▼
                                        StoreAndForward (priority queue, batching,
                                        retry + exponential back-off) ─► ITransport
                                                                      (MQTT / HTTPS / file)
```

## Modules

| File | Purpose |
|---|---|
| `obd2.*` | Build ISO-TP single-frame Mode 01 requests (`0x7DF`), decode responses (`0x7E8`–`0x7EF`): speed, RPM, coolant, throttle, fuel, load, intake temp, battery voltage. Rejects negative / short / multi-frame responses. |
| `gps.*` | NMEA `RMC` / `GGA` parsing with checksum validation (any talker: GP, GN, GL…), hemisphere handling, haversine distance. |
| `event_detector.*` | Harsh braking / acceleration (from speed derivative, debounced per manoeuvre), overspeed, excessive idle, overheat, low fuel, low battery. Level events fire once and re-arm when cleared. |
| `trip_tracker.*` | Trip start on ignition, end after a configurable engine-off period; distance (GPS with jump rejection, or wheel-speed integration), max/avg speed, idle time, fuel used, harsh-event count. |
| `store_forward.*` | Thread-safe bounded queue. Alerts jump the queue; when full the oldest normal message is evicted first. Messages are only removed after the transport acknowledges them. |
| `can_bus.*` | `ICanBus` with a real Linux **SocketCAN** implementation (kernel filter for OBD response IDs) and a **simulated ECU** driven by a physical drive-cycle model. |
| `telematics_unit.*` | Orchestrator: polls PIDs, ingests NMEA, runs detection and trips, publishes JSON (periodic while driving, sparse while parked). |

### Networking (`include/telematics/net/`, `src/net/`)

| File | Purpose |
|---|---|
| `stream.hpp` | `IStream` interface shared by TCP and TLS, so MQTT and HTTP run over either. Typed errors: `NetError`, `TimeoutError`, `ProtocolError`. |
| `tcp_socket.*` | TCP/IP client and listener: DNS (IPv4/IPv6), connect timeout via non-blocking connect + `poll`, timeouts on every read/write, `TCP_NODELAY`, keep-alive tuned to detect a dead cellular link in ~60 s, no SIGPIPE. |
| `tls.*` | OpenSSL TLS 1.2/1.3 client and server: CA from system store / file / PEM string, SNI, hostname and IP-SAN verification, client certificates (mutual TLS for AWS IoT, Azure IoT Hub), ALPN, clear verification error messages. |
| `mqtt_codec.*` | MQTT 3.1.1 packet encoder/decoder and incremental framer, written from the OASIS spec: CONNECT (auth, last will), PUBLISH, PUBACK, SUBSCRIBE, UNSUBSCRIBE, PING, DISCONNECT; validates topics, QoS and reserved flags. |
| `mqtt_client.*` | Synchronous, thread-safe client: QoS 0/1 publish, subscribe with message callback, keep-alive with dead-broker detection, graceful disconnect. |
| `http_client.*` | HTTP/1.1 client over TCP or TLS: URL parsing (incl. IPv6), Content-Length, chunked and read-to-close bodies, 1xx handling, size limits, header-injection protection. |
| `transports.*` | `MqttTransport` (topics `vehicles/<vin>/{telemetry,event,trip}` plus a retained `status` online/offline via last will) and `HttpsTransport` (JSON-array batches, bearer token; retries 408/429/5xx, drops other 4xx so one bad batch cannot block the queue). |

## Build and run

```bash
cmake -S . -B build && cmake --build build -j
./build/tcu_tests          # vehicle-side unit tests
./build/tcu_net_tests      # network tests: codec + MQTT/HTTPS over real TLS on loopback
./build/tcu_app            # 13-minute simulated drive incl. a cellular dead zone
```

Requires CMake 3.14+, a C++17 compiler and OpenSSL 1.1.1+ development headers
(`sudo apt install libssl-dev` on Debian/Ubuntu).

### Streaming to a real backend

```bash
# MQTT over TLS with a device certificate (mutual TLS), e.g. AWS IoT Core
./build/tcu_app --uplink mqtts://<endpoint>:8883 --ca AmazonRootCA1.pem \
                --cert device.pem.crt --key private.pem.key --vin MA3EWDE1S00000000

# MQTT with username/token auth
./build/tcu_app --uplink mqtts://broker.example.com:8883 --token <password-or-JWT>

# HTTPS ingest endpoint with a bearer token (system CA store unless --ca is given)
./build/tcu_app --uplink https://api.example.com/v1/ingest --token <bearer>
```

Without `--uplink` the app writes to `telemetry.jsonl`. All options also work in live
`--can` mode. For a quick local broker: `mosquitto -p 1883`, then `--uplink mqtt://localhost:1883`
(plain TCP: for testing only).

The simulation writes `telemetry.jsonl` and prints events, e.g.

```
[event] {"type":"event","event":"harsh_braking","value":-8.06,...}
[trip]  {"type":"trip","distance_km":7.16,"max_speed_kmh":119.00,"idle_s":388.00,...}
```

### Live mode on a real vehicle or a virtual CAN bus (Linux)

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
./build/tcu_app --can vcan0 --nmea /dev/ttyACM0 --seconds 120
```

With a USB-CAN adapter on the OBD port (500 kbit/s, 11-bit), use `can0` instead:
`sudo ip link set can0 up type can bitrate 500000`.

## Extending

- **Commands to the vehicle**: `MqttClient::subscribe("vehicles/<vin>/cmd")` and call `loop()`
  periodically from the TCU thread; handle messages in `onMessage`.
- **HTTP keep-alive / HTTP/2**: the HTTP client opens one connection per batch, which is
  simple and robust but costs a TLS handshake each time; reuse connections for high rates.
- **Non-blocking DNS**: `getaddrinfo` blocks; use c-ares or a resolver thread on constrained targets.
- **Persistence across reboots**: back the store-and-forward queue with a file or SQLite.
- **More PIDs / J1939**: add cases to `obd2::decodeResponse`, or a J1939 decoder for trucks
  (29-bit IDs are already supported by `SocketCanBus`).
- **Multi-frame responses (VIN, DTCs)**: add ISO-TP flow control (first frame / consecutive frames).

## Verification

- All suites pass under AddressSanitizer + UBSan and ThreadSanitizer; release build has zero warnings.
- Interoperability checked against independent implementations: the simulated drive streamed
  to the amqtt broker over TLS (all messages received by a separate subscriber; last will
  fires on `kill -9` and not on clean exit) and to a Python `http.server` HTTPS endpoint.
