## 📂 Detailed File Description

The project contains Python programs for system-level processing and visualization, along with Arduino/ESP32 firmware for implementing the LoRa communication system.

### 1. `Emergency_mode.py`

This Python file implements the **Emergency Mode** of the communication system.

When an emergency condition is detected, the system switches from normal operation to emergency handling. The purpose of this mode is to ensure that emergency-related information can be processed and communicated with appropriate priority.

**Main responsibilities:**

* Handles emergency-mode operation.
* Processes emergency-related data.
* Supports priority communication during emergency conditions.
* Interfaces with the communication workflow implemented by the project.

**Role in the system:**

```text
Emergency Condition
        ↓
Emergency_mode.py
        ↓
Emergency Data Processing
        ↓
LoRa Communication
```

---

### 2. `Normal_Mode.py`

This file implements the **Normal Mode** of the system.

Normal Mode is used when the system is operating under regular conditions and there is no emergency event requiring priority handling.

**Main responsibilities:**

* Handles normal system operation.
* Processes regular communication data.
* Controls the normal communication workflow.
* Provides the standard operating path before an emergency condition occurs.

**Role in the system:**

```text
Normal Input
     ↓
Normal_Mode.py
     ↓
Data Processing
     ↓
LoRa Transmission
```

---

### 3. `Load_part.py`

`Load_part.py` is responsible for the **load-related processing** used within the project.

The load-processing component is important because the system needs to determine how data or communication resources should be handled under different conditions.

**Main responsibilities:**

* Processes load-related information.
* Performs calculations required by the load-handling section.
* Provides processed information to other parts of the system.
* Supports the overall communication/decision-making process.

**Role in the system:**

```text
Input / Load Data
       ↓
Load Processing
       ↓
Load_part.py
       ↓
Processed Result
```

---

### 4. `best_weight.py`

`best_weight.py` contains the **weight-related calculation/processing logic** used by the project.

The file is associated with determining or working with suitable weights used in the project's processing or decision-making mechanism.

**Main responsibilities:**

* Performs weight-related calculations.
* Processes available input parameters.
* Determines/handles weight values required by the system.
* Supports the project's data-processing or decision mechanism.

**Role in the system:**

```text
Input Parameters
       ↓
Weight Calculation
       ↓
best_weight.py
       ↓
Selected / Processed Weight
       ↓
System Decision
```

---

### 5. `visualizer.py`

`visualizer.py` is the **visualization component** of the project.

Instead of relying only on raw terminal or serial output, this component provides a way to represent system information graphically or in an easier-to-understand format.

**Main responsibilities:**

* Reads/handles system data for visualization.
* Displays communication or processing information.
* Helps monitor system behavior.
* Supports testing and demonstration.
* Makes it easier to analyze system performance.

**Role in the system:**

```text
System Data
     ↓
visualizer.py
     ↓
Data Visualization
     ↓
Monitoring / Analysis
```

---

### 6. Arduino `.ino` Files

The repository also contains multiple `.ino` files. These files contain the **embedded firmware executed on the ESP32/LoRa hardware**.

The Arduino programs form the hardware-side implementation of the communication system.

Depending on the specific `.ino` file, the firmware may be responsible for:

* ESP32 initialization
* LoRa module initialization
* Transmitting data
* Receiving data
* Serial communication
* Reading input/sensor data
* Handling communication modes
* Processing received packets
* Sending information between nodes

The general firmware workflow is:

```text
ESP32 Startup
      ↓
Initialize Hardware
      ↓
Initialize LoRa
      ↓
Read / Process Data
      ↓
Transmit or Receive
      ↓
Handle Communication
      ↓
Repeat
```

The `.ino` files therefore represent the **embedded/hardware layer** of the project, while the Python files primarily provide the higher-level processing and visualization functionality.

---

### 7. `Reciver.txt`

`Reciver.txt` contains receiver-related material associated with the project.

It is used as a reference for the **receiver side of the communication system** and can contain receiver implementation or supporting information required during development.

The receiver's general responsibility is:

```text
LoRa Signal
     ↓
Receiver / ESP32
     ↓
Packet Reception
     ↓
Data Processing
     ↓
Output / Visualization
```

> The filename `Reciver.txt` is retained as it exists in the repository. For a polished submission, it would be preferable to rename it to `Receiver.txt` if it is safe to do so without breaking project references.

---

## 🔗 How the Files Work Together

The files can be viewed as different layers of the overall system:

```text
                 ┌─────────────────────────┐
                 │       INPUT DATA        │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │    Python Processing    │
                 │                         │
                 │ Normal_Mode.py          │
                 │ Emergency_mode.py       │
                 │ Load_part.py            │
                 │ best_weight.py          │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │   ESP32 / LoRa Layer    │
                 │                         │
                 │ Arduino .ino files      │
                 └────────────┬────────────┘
                              │
                         LoRa Link
                              │
                              ▼
                 ┌─────────────────────────┐
                 │    Receiver System      │
                 │                         │
                 │ Receiver implementation │
                 │ / ESP32 firmware        │
                 └────────────┬────────────┘
                              │
                              ▼
                 ┌─────────────────────────┐
                 │     Visualization       │
                 │                         │
                 │     visualizer.py       │
                 └─────────────────────────┘
```

## 🧩 File Responsibilities at a Glance

| File                | Layer                    | Primary Purpose                         |
| ------------------- | ------------------------ | --------------------------------------- |
| `Emergency_mode.py` | Application / Processing | Emergency-mode operation                |
| `Normal_Mode.py`    | Application / Processing | Normal-mode operation                   |
| `Load_part.py`      | Processing               | Load-related calculations/processing    |
| `best_weight.py`    | Processing / Algorithm   | Weight-related calculations             |
| `visualizer.py`     | Visualization            | Monitoring and graphical representation |
| `.ino` files        | Embedded / Hardware      | ESP32 and LoRa firmware                 |
| `Reciver.txt`       | Receiver                 | Receiver-side reference/implementation  |

## 🏛️ Overall Software-Hardware Relationship

The project can therefore be divided into three major layers:

### Application Layer

Python programs control the higher-level processing and operating modes.

```text
Emergency_mode.py
Normal_Mode.py
Load_part.py
best_weight.py
```

### Communication & Embedded Layer

ESP32 and LoRa firmware provides the actual wireless communication between hardware nodes.

```text
Arduino .ino files
```

### Monitoring Layer

The visualization component provides a human-readable representation of system information.

```text
visualizer.py
```

Together, these components form the software and embedded architecture of the **LoRa Based Communication System**.
