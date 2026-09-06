# 📡 Energy-Aware Greedy Relay Mesh Network over LoRa

**Hybrid RSSI–Battery Greedy Relay Selection for Energy-Efficient Multi-Hop LoRa Mesh Networks in Disaster Scenarios**

A multi-hop **LoRa mesh network** built on **ESP32 / ESP8266 nodes**, where every node picks the *next hop* on the fly using a **greedy relay score that blends link quality (RSSI) with the relay's remaining battery**. The goal is a mesh that keeps routing data after a disaster knocks out cellular/Wi-Fi infrastructure — without draining the first "good signal" node to zero while idle nodes sit at full charge.

This repository contains the **firmware** (Arduino `.ino` for ESP32/ESP8266 + LoRa), the **Python simulators** used to design and validate the routing algorithm before flashing hardware, and the **Monte Carlo sweep results** used to tune the algorithm's weighting.



---

## 📌 Table of Contents

* [Project Overview](#-project-overview)
* [How Relay Selection Works](#-how-relay-selection-works)
* [Probe-Then-Send Protocol](#-probe-then-send-protocol)
* [Repository Structure](#-repository-structure)
* [Firmware Files](#-firmware-files)
* [Known Bugs Fixed in `_FIXED` Firmware](#-known-bugs-fixed-in-_fixed-firmware)
* [Python Simulation & Tooling](#-python-simulation--tooling)
* [Hardware Requirements](#-hardware-requirements)
* [Wiring](#-wiring)
* [Software Requirements & Installation](#-software-requirements--installation)
* [Flashing a Mesh Node](#-flashing-a-mesh-node)
* [Running the Python Tools](#-running-the-python-tools)
* [Simulation Results](#-simulation-results)
* [Applications](#-applications)
* [Limitations](#-limitations)
* [Future Improvements](#-future-improvements)
* [License](#-license)

---

# 📌 Project Overview

Standard LoRa "shortest path / strongest signal" routing tends to keep funneling traffic through whichever node happens to have the best RSSI — which quietly drains that node's battery while its neighbors idle. This project instead scores every candidate relay with a **weighted combination of normalized RSSI and normalized remaining battery**:

```
score = w1 * normalized_RSSI + w2 * normalized_battery
```

`w1` and `w2` are tuned via Monte Carlo simulation (see [`best_weight.py`](#-python-simulation--tooling) and [`lora_ai_optimizer.py`](#-python-simulation--tooling)) rather than guessed, trading a small amount of raw packet delivery ratio for a much longer overall network lifetime.

```text
┌───────────────────────────────────────────────┐
│                APPLICATION LAYER               │
│   Weight tuning (Monte Carlo) · Visualization  │
│        best_weight.py · lora_ai_optimizer.py   │
│                   visualizer.py                │
└───────────────────────┬─────────────────────────┘
                        │
                        ▼
┌───────────────────────────────────────────────┐
│           EMBEDDED / MESH ROUTING LAYER        │
│   ESP32 / ESP8266 + SX127x LoRa module         │
│   Neighbor table · Greedy scoring · TX queue   │
│   node_TEMPLATE_esp32.ino / esp8266.ino        │
└───────────────────────┬─────────────────────────┘
                        │
                   LoRa Wireless
                   Multi-Hop Link
                        │
                        ▼
┌───────────────────────────────────────────────┐
│              DESTINATION / LOGGING             │
│      Serial output · result_*.csv logging      │
└───────────────────────────────────────────────┘
```

---

# 🧠 How Relay Selection Works

Each node keeps a small **neighbor table** (`Neighbor{ id, rssi, battery, hop, last_seen, score }`) built from HELLO exchanges. When a node needs to send data to a destination it doesn't have a direct link's worth of confidence in, it:

1. Normalizes the neighbor's RSSI between `RSSI_MIN (-120 dBm)` and `RSSI_MAX (-40 dBm)`.
2. Normalizes the neighbor's last-reported battery percentage.
3. Computes `score = W1*rssi_norm + W2*battery_norm` (default `W1=0.6`, `W2=0.4`).
4. Picks the highest-scoring neighbor as the next hop, subject to `SCORE_THRESHOLD` and `MAX_HOPS`.

Every node also runs a simple **energy model** locally (`TX_COST`, `RX_COST`, `IDLE_DRAIN`) so its self-reported battery level actually depletes over the simulation/demo, which keeps the greedy score honest during long-running tests.

---

# 🔁 Probe-Then-Send Protocol

Rather than blindly trusting stale neighbor-table entries, every send does a fresh probe first:

```text
1. [INIT] Node ready, message typed (or "<DEST_ID>:<message>" on template firmware)
2. [TX]   Send probe HELLO to destination
3. [WAIT] Wait up to HELLO_REPLY_WAIT (3s) for a fresh HELLO reply carrying live RSSI
4. [CALC] Run greedy score against the neighbor table
5. [TX]   Send DATA packet via the chosen relay
6. [RX]   Receiver logs: "Received <msg> | Greedy score: X"
```

If no reply arrives in time, the node falls back to its cached neighbor-table entry (if any) or drops the send and retries with jittered backoff (`MIN_BACKOFF_MS`–`MAX_BACKOFF_MS`) to avoid collisions on shared airtime.

---

# 📁 Repository Structure

```text
Major-Project/
│
├── Firmware (Arduino / .ino)
│   ├── transmitter.ino              # earliest single-link TX-only test
│   ├── receiver.ino                 # earliest single-link RX-only test (+ VCC/battery sag logging)
│   ├── 1.ino                        # early two-node HELLO exchange test
│   ├── NodeA_FINAL.ino              # early full relay firmware (source role)
│   ├── NodeB_final.ino              # early full relay firmware (intermediate relay role)
│   ├── NodeD_FINAL.ino              # early full relay firmware (destination role)
│   ├── node_A_esp32_FIXED.ino       # Node A, bug-fixed (see below)
│   ├── node_D_esp8266_FIXED.ino     # Node D, bug-fixed (see below)
│   ├── node_TEMPLATE_esp32.ino      # generalized N-node template, ESP32
│   └── node_TEMPLATE_esp8266.ino    # generalized N-node template, ESP8266
│
├── Python Simulation & Tooling
│   ├── Normal_Mode.py               # early single-hop Tkinter/networkx demo (normal routing)
│   ├── Emergency_mode.py            # early single-hop Tkinter/networkx demo (priority routing)
│   ├── Load_part.py                 # early animated multi-node Tkinter/networkx load demo
│   ├── best_weight.py               # vectorized numpy/multiprocessing sweep of w1 (RSSI weight)
│   ├── visualizer.py                # scalable (10–10,000+ node) multi-hop mesh simulator + GUI
│   └── lora_ai_optimizer.py         # Monte Carlo weight optimizer + sklearn emergency profiler + GUI
│
├── Data
│   └── result_02092026.csv          # Monte Carlo sweep output (w1/w2 vs PDR, energy, hops, etc.)
│
├── Reciver.txt                      # reference copy of an early receiver sketch
├── requirements.txt                 # Python dependencies
└── README.md
```

---

# 🔌 Firmware Files

| File | Role | Board | Notes |
|---|---|---|---|
| `transmitter.ino` | TX-only test | ESP32 | Sends a counting `<HELLO n>` packet every 2s. First hardware smoke test. |
| `receiver.ino` | RX-only test | ESP8266 | Logs RSSI and VCC sag (`ADC_MODE(ADC_VCC)`) on every received packet — used to sanity-check the battery-drain-under-TX assumption. |
| `1.ino` | Early two-node test | ESP8266 | First HELLO ping/reply exchange between two boards. |
| `NodeA_FINAL.ino` / `NodeB_final.ino` / `NodeD_FINAL.ino` | Full relay firmware, fixed roles | ESP32/ESP8266 | Neighbor table, TX queue, message-dedup cache, greedy scoring — but with hardcoded roles (superseded by the `_FIXED` and `TEMPLATE` firmware below). |
| `node_A_esp32_FIXED.ino` | Source node, bugs fixed | ESP32 | See [bug fixes](#-known-bugs-fixed-in-_fixed-firmware). |
| `node_D_esp8266_FIXED.ino` | Destination node, bugs fixed | ESP8266 | See [bug fixes](#-known-bugs-fixed-in-_fixed-firmware). |
| `node_TEMPLATE_esp32.ino` | **Current mesh firmware** | ESP32 | Flash the *same* file to every ESP32 board; only `NODE_ID` changes per board. Supports `<DEST_ID>:<message>` addressing to any node in the mesh. |
| `node_TEMPLATE_esp8266.ino` | **Current mesh firmware** | ESP8266 | Same idea as above, for ESP8266 boards. |

For a small demo, flash the two `_FIXED` files. For an actual N-node mesh, flash `node_TEMPLATE_esp32.ino` / `node_TEMPLATE_esp8266.ino` to every board and give each one a unique `NODE_ID`.

---

# 🐛 Known Bugs Fixed in `_FIXED` Firmware

While debugging the two-node prototype, two issues were found and fixed:

1. **Node A was probing itself.** `node_A_esp32_FIXED.ino`'s `read_serial()` was copy-pasted from Node D's code, and the hardcoded probe destination was never updated from `"A"` to `"D"`. Node A was sending HELLO probes to its own node ID and never actually reaching the destination.
2. **HELLO ping-pong starving the channel.** Both nodes replied to *every* incoming HELLO unconditionally — including replies to their own probes — which caused an infinite HELLO↔HELLO exchange that ate all the available airtime and starved real DATA packets. The fix adds a reply-flag to the HELLO packet so a node only auto-replies to a genuine probe, never to a reply to its own probe.

Both fixes are also carried forward into `node_TEMPLATE_esp32.ino` / `node_TEMPLATE_esp8266.ino`.

---

# 🐍 Python Simulation & Tooling

| File | What it does |
|---|---|
| `Normal_Mode.py` | Early single-hop LoRa demo (Tkinter GUI + `networkx` topology view) — standard-priority routing baseline used before the greedy relay logic existed. |
| `Emergency_mode.py` | Same demo as above, adapted for priority/emergency-flagged traffic. |
| `Load_part.py` | Animated multi-node Tkinter/`networkx` load-handling demo (`matplotlib.animation`) — visualizes how load is distributed across a small network. |
| `best_weight.py` | Sweeps the RSSI weight `w1` from 0→1 in steps of 0.01 (`w2 = 1 - w1`) to find the weighting that best balances PDR, network lifetime, and end-to-end delay. Vectorized with numpy and parallelized across CPU cores with `multiprocessing.Pool` (a rewrite of an earlier MATLAB script). |
| `visualizer.py` | The main **scalable multi-hop mesh simulator** — supports 10 to 10,000+ simulated nodes with a live GUI, CSMA/CA collision modeling, an energy model, and CSV logging of every run. The core `compute_rssi()`, `greedy_neighbor()`, and `custom_greedy()` routing functions are kept byte-identical to the original algorithm so simulation results stay comparable across versions. |
| `lora_ai_optimizer.py` | The most advanced tool: a 4-tab Tkinter GUI (**Optimizer / Emergency / Results / Network**) that runs a full **Monte Carlo engine** across randomized area, battery, node count, and deployment, does a coarse 10×10 grid search followed by a fine random search over `(w1, w2)`, and trains a small `sklearn` `MLPRegressor` on the resulting dataset to extract 3 emergency-traffic profiles via K-Means. |

`result_02092026.csv` is a sample output of the Monte Carlo sweep — each row is one `(w1, w2)` candidate with its mean PDR, energy use, collision rate, channel utilization, hop count, reach rate, remaining battery, RSSI, and neighbor count across all trials.

---

# 🔧 Hardware Requirements

| Component | Purpose |
|---|---|
| ESP32 dev board | Mesh node (VSPI wiring, see below) |
| ESP8266 dev board (e.g. NodeMCU / Wemos D1 mini) | Mesh node (lower-cost alternative role) |
| SX127x / Ra-02 LoRa module | 433 MHz LoRa radio |
| Antenna (433 MHz) | Matched to the module/region |
| USB cables + power source per node | Node power |
| Computer | Flashing firmware, running the Python simulators/optimizer |

> ⚠️ 433 MHz is used throughout (`FREQUENCY 433E6`) — confirm this is legal in your region before transmitting, and adjust if not.

---

# 🔌 Wiring

**ESP32 (VSPI) — `node_TEMPLATE_esp32.ino` / `node_A_esp32_FIXED.ino`:**

| LoRa Pin | ESP32 GPIO |
|---|---|
| NSS / CS | 5 |
| RST | 14 |
| DIO0 | 26 |
| SCK | 18 |
| MISO | 19 |
| MOSI | 23 |

**ESP8266 — `node_TEMPLATE_esp8266.ino` / `node_D_esp8266_FIXED.ino`:**

| LoRa Pin | ESP8266 Pin |
|---|---|
| NSS / CS | 15 (D8) |
| RST | 5 (D1) |
| DIO0 | 4 (D2) |

(SPI SCK/MISO/MOSI use the ESP8266's default hardware SPI pins.)

---

# 💻 Software Requirements & Installation

* **Arduino IDE** with ESP32 and ESP8266 board packages installed
* The [`LoRa` Arduino library](https://github.com/sandeepmistry/arduino-LoRa) (Sandeep Mistry) — used via `#include <LoRa.h>` and `#include <SPI.h>`
* **Python 3.x** for the simulation/optimizer tools

Clone the repo:

```bash
git clone https://github.com/Aadit2308/Major-Project.git
cd Major-Project
```

Install Python dependencies:

```bash
pip install -r requirements.txt
```

`requirements.txt` currently lists: `numpy`, `pandas`, `matplotlib`, `pyserial`. For the fuller feature set in `visualizer.py` and `lora_ai_optimizer.py`, also install the optional extras they auto-detect at import time:

```bash
pip install scipy scikit-learn networkx
```

(Both scripts degrade gracefully — `scipy`'s KD-tree speeds up neighbor lookups but isn't required, and `sklearn` is only needed for the Emergency Profiler tab in `lora_ai_optimizer.py`.)

---

# ⚡ Flashing a Mesh Node

1. Open `node_TEMPLATE_esp32.ino` (for ESP32 boards) or `node_TEMPLATE_esp8266.ino` (for ESP8266 boards) in Arduino IDE.
2. Change **only** the `NODE_ID` line to a unique single letter for this board (`"A"`, `"B"`, `"C"`, `"D"`, ...). Nothing else needs editing.
3. Wire the LoRa module as described above.
4. Select the correct board and serial port in Arduino IDE.
5. Upload.
6. Open the Serial Monitor at `115200` baud.
7. To send a message to another node, type `<DEST_ID>:<message>` and press Enter — e.g. `C:sensor reading is 42` sends to Node C, hopping through relays as needed.
8. An idle node stays in pure-RX mode and only transmits when you send, when relaying someone else's traffic, or when replying to a genuine HELLO probe.

Repeat for every board in the mesh, giving each one a different `NODE_ID`.

---

# 🖥️ Running the Python Tools

```bash
# Weight sweep (best RSSI vs. battery weighting)
python best_weight.py

# Scalable multi-hop mesh simulator with GUI
python visualizer.py

# Monte Carlo weight optimizer + emergency profiler GUI
python lora_ai_optimizer.py
```

All three write/consume CSV logs (see `result_02092026.csv` for the expected sweep-output format) so runs can be compared across sessions without re-running the full Monte Carlo pass each time.

---

# 📊 Simulation Results

`result_02092026.csv` records, per `(w1, w2)` candidate, the mean across all Monte Carlo trials for: **PDR**, **energy consumption**, **collision rate**, **channel utilization**, **hop count**, **reach rate**, **remaining battery**, **RSSI**, and **neighbor count**. This is the dataset used to pick the default `W1 = 0.6` / `W2 = 0.4` weighting baked into the firmware.

---

# 🔬 Applications

* Disaster-response and search-and-rescue communication
* Off-grid sensor networks where node lifetime matters more than raw throughput
* Rural or infrastructure-poor area monitoring
* Any scenario needing multi-hop coverage extension beyond a single LoRa node's range

---

# ⚠️ Limitations

* LoRa's low data rate limits this to small control/telemetry packets, not bulk data.
* Greedy (locally-optimal) relay selection can still make globally suboptimal choices compared to a full routing-table approach.
* `HELLO_REPLY_WAIT` (3s) and backoff windows trade responsiveness for collision avoidance — tune per deployment density.
* 433 MHz operation and TX power must comply with local spectrum regulations.
* Firmware assumes a single-letter `NODE_ID` namespace (`MAX_NEIGHBORS` currently capped at 16 on the ESP32 template).

---

# 🔮 Future Improvements

* Merge the `_FINAL` / `_FIXED` firmware history into a single maintained template (currently kept for reference/debugging history)
* Persist battery/energy state across reboots (currently resets to `INITIAL_BATTERY` on power-cycle)
* Field-test the Monte-Carlo-derived weights against the physical mesh and feed real RSSI/PDR data back into `lora_ai_optimizer.py`
* Encryption/authentication on HELLO and DATA packets
* Automatic retransmission/ACK layer for lost DATA packets
* Cloud/dashboard logging when backhaul is available

---

# 📜 License

This project is intended primarily for **academic and educational purposes** (VESIT E&TC major project). If it's reused or redistributed publicly, add an appropriate open-source license to the repository.

---

# ⭐ Acknowledgements

Developed as a VESIT Electronics & Telecommunication Engineering major project by **Aadit** accompanying the paper *"Hybrid RSSI–Battery Greedy Relay Selection for Energy-Efficient Multi-Hop LoRa Mesh Networks in Disaster Scenarios."*
