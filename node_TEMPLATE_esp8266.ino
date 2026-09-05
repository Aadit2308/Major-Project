// ============================================================
//  ENERGY-AWARE GREEDY RELAY — MESH NODE (ESP8266 TEMPLATE)
//  VESIT E&TC — Aadit | Co-authors: Samiksha, Anushree
//
//  N-NODE TEMPLATE: flash this SAME file to every ESP8266 board
//  in your mesh. The ONLY thing you change per board is the
//  NODE_ID line right below. Nothing else needs editing.
//
//  Usage: type  <DEST_ID>:<message>  then Enter
//         e.g.  A:sensor reading is 42     -> sends to Node A
//
//  IDLE = pure RX, nothing transmitted until you send.
//  On send:
//    1. Parse "<DEST>:<msg>" from serial input
//    2. Send probe HELLO to DEST
//    3. Wait up to 3s for DEST's HELLO reply (fresh RSSI)
//    4. Run greedy relay score -> [TX] DATA sent
//  On receiver: [RX] MESSAGE RECEIVED banner with score/route
//  On relay: multi-hop forwarding via greedy neighbor scoring
// ============================================================

#include <SPI.h>
#include <LoRa.h>

// ─────────────────────────────────────────────────────────
//  >>> CHANGE ONLY THIS PER BOARD <<<
// ─────────────────────────────────────────────────────────
#define NODE_ID  "D"   // Unique single-letter ID: "A","B","C","D","E"...

#define NSS   15   // D8
#define RST   5    // D1
#define DIO0  4    // D2

#define MSG_CACHE_SIZE      40    // bumped for more nodes/traffic
#define TX_QUEUE_SIZE       16    // bumped — relay hubs queue more
#define MIN_BACKOFF_MS      100   // widened backoff — more nodes = more collisions
#define MAX_BACKOFF_MS      400
#define TX_COOLDOWN_MS      150
#define HELLO_REPLY_WAIT    3000

#define INITIAL_BATTERY       100.0
#define LOW_BATTERY_THRESHOLD 20.0
#define TX_COST               0.5
#define RX_COST               0.2
#define IDLE_DRAIN            0.01

#define FREQUENCY        433E6
#define SPREADING_FACTOR 7
#define BANDWIDTH        125E3
#define CODING_RATE      5
#define MAX_HOPS         8      // raised — allow longer multi-hop chains
#define SCORE_THRESHOLD  0.15
#define RSSI_MIN        -120.0
#define RSSI_MAX        -40.0
#define W1               0.6
#define W2               0.4

struct Neighbor {
  String id; int rssi; float battery; int hop;
  unsigned long last_seen; float score;
};
struct TxEntry { String packet; bool valid; };

#define MAX_NEIGHBORS 16   // bumped for larger mesh
Neighbor neighbor_table[MAX_NEIGHBORS];
int neighbor_count = 0;

TxEntry tx_queue[TX_QUEUE_SIZE];
int tx_queue_head = 0, tx_queue_tail = 0, tx_queue_size = 0;

String recent_msg_cache[MSG_CACHE_SIZE];
int cache_head = 0, cache_size = 0;

float         current_battery   = INITIAL_BATTERY;
unsigned long last_drain_time   = 0;
unsigned long last_tx_time      = 0;
unsigned long backoff_until     = 0;
String        serial_buf        = "";

String        pending_dest      = "";
String        pending_msg       = "";
bool          waiting_for_reply = false;
unsigned long hello_sent_at     = 0;

// ═══════════════════════════════════════════════════════
//  HELPERS
// ═══════════════════════════════════════════════════════

float normalize_rssi(int r) {
  float n = (r - RSSI_MIN) / (RSSI_MAX - RSSI_MIN);
  return n < 0.0 ? 0.0 : (n > 1.0 ? 1.0 : n);
}
float normalize_battery(float b) {
  float n = b / 100.0;
  return n < 0.0 ? 0.0 : (n > 1.0 ? 1.0 : n);
}
float compute_score(int r, float b) {
  return (W1 * normalize_rssi(r)) + (W2 * normalize_battery(b));
}
void clamp_battery() {
  if (current_battery < 0.0)   current_battery = 0.0;
  if (current_battery > 100.0) current_battery = 100.0;
}
bool in_cache(String id) {
  if (id == "" || id == "0") return false;
  for (int i = 0; i < cache_size; i++)
    if (recent_msg_cache[i] == id) return true;
  return false;
}
void add_to_cache(String id) {
  if (id == "" || id == "0") return;
  recent_msg_cache[cache_head] = id;
  cache_head = (cache_head + 1) % MSG_CACHE_SIZE;
  if (cache_size < MSG_CACHE_SIZE) cache_size++;
}

// ═══════════════════════════════════════════════════════
//  TX QUEUE
// ═══════════════════════════════════════════════════════

bool enqueue_tx(String pkt) {
  if (tx_queue_size >= TX_QUEUE_SIZE) {
    Serial.println("[QUEUE] FULL — dropped");
    return false;
  }
  tx_queue[tx_queue_head] = {pkt, true};
  tx_queue_head = (tx_queue_head + 1) % TX_QUEUE_SIZE;
  tx_queue_size++;
  return true;
}
String dequeue_tx() {
  if (tx_queue_size == 0) return "";
  String p = tx_queue[tx_queue_tail].packet;
  tx_queue[tx_queue_tail].valid = false;
  tx_queue_tail = (tx_queue_tail + 1) % TX_QUEUE_SIZE;
  tx_queue_size--;
  return p;
}

// ═══════════════════════════════════════════════════════
//  PROCESS TX QUEUE
// ═══════════════════════════════════════════════════════

void process_tx_queue() {
  if (tx_queue_size == 0) return;
  unsigned long now = millis();
  if (now < backoff_until) return;
  if ((now - last_tx_time) < TX_COOLDOWN_MS) return;

  String pkt = dequeue_tx();
  if (pkt == "") return;

  LoRa.beginPacket();
  LoRa.print(pkt);
  LoRa.endPacket(false);

  last_tx_time  = millis();
  backoff_until = millis() + random(MIN_BACKOFF_MS, MAX_BACKOFF_MS);
  current_battery -= TX_COST;
  clamp_battery();

  if (pkt.startsWith("DATA")) {
    String f[8]; int fi = 0; String tok = "";
    for (int i = 0; i < (int)pkt.length() && fi < 7; i++) {
      if (pkt[i] == '|') { f[fi++] = tok; tok = ""; } else tok += pkt[i];
    }
    f[fi] = tok;
    float relay_score = (neighbor_count > 0) ? neighbor_table[0].score : 0.0;
    for (int i = 0; i < neighbor_count; i++) {
      if (neighbor_table[i].id == f[2] || neighbor_count == 1) {
        relay_score = neighbor_table[i].score;
        break;
      }
    }
    Serial.println("┌─────────────────────────────────────");
    Serial.println("│ [TX] Sent toward Node " + f[2]);
    Serial.println("│ Msg        : \"" + f[6] + "\"");
    Serial.println("│ Greedy Score: " + String(relay_score, 4));
    Serial.println("│ Route      : " + f[7]);
    Serial.println("│ Batt       : " + String(current_battery, 1) + "%");
    Serial.println("└─────────────────────────────────────");
  }
}

// ═══════════════════════════════════════════════════════
//  NEIGHBOR TABLE
// ═══════════════════════════════════════════════════════

void update_neighbor(String id, int rssi, float battery, int hop) {
  for (int i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].id == id) {
      neighbor_table[i] = {id, rssi, battery, hop, millis(), compute_score(rssi, battery)};
      return;
    }
  }
  if (neighbor_count < MAX_NEIGHBORS) {
    float s = compute_score(rssi, battery);
    neighbor_table[neighbor_count] = {id, rssi, battery, hop, millis(), s};
    Serial.println("[NBR] New neighbor: " + id +
                   " | RSSI:" + rssi +
                   " | Batt:" + String(battery, 1) +
                   " | Score:" + String(s, 4));
    neighbor_count++;
  }
}

// ═══════════════════════════════════════════════════════
//  GREEDY SELECTION
// ═══════════════════════════════════════════════════════

int select_best_neighbor(String exclude_id) {
  float best = -9999.0; int idx = -1;

  Serial.println("┌─────────────────────────────────────");
  Serial.println("│ [GREEDY] Evaluating neighbors:");
  for (int i = 0; i < neighbor_count; i++) {
    if (neighbor_table[i].id == exclude_id) continue;
    if (neighbor_table[i].battery < LOW_BATTERY_THRESHOLD) {
      Serial.println("│   SKIP " + neighbor_table[i].id + " — low battery");
      continue;
    }
    float s = compute_score(neighbor_table[i].rssi, neighbor_table[i].battery);
    neighbor_table[i].score = s;
    Serial.println("│   Node " + neighbor_table[i].id +
                   " | RSSI:" + String(neighbor_table[i].rssi) +
                   " | Batt:" + String(neighbor_table[i].battery, 1) +
                   " | Score:" + String(s, 4));
    if (s > best) { best = s; idx = i; }
  }
  if (idx == -1 || best < SCORE_THRESHOLD) {
    Serial.println("│   No valid relay found");
    Serial.println("└─────────────────────────────────────");
    return -1;
  }
  Serial.println("│ >>> Best relay: Node " + neighbor_table[idx].id +
                 " | Score:" + String(best, 4));
  Serial.println("└─────────────────────────────────────");
  return idx;
}

// ═══════════════════════════════════════════════════════
//  PROBE — send HELLO then wait for reply
// ═══════════════════════════════════════════════════════

void probe_and_send(String dest, String message) {
  pending_dest      = dest;
  pending_msg       = message;
  waiting_for_reply = true;
  hello_sent_at     = millis();

  // HELLO format: HELLO|src|battery|hop|is_reply
  // is_reply = 0 -> this is an original probe, please reply
  String hello = "HELLO|" + String(NODE_ID) + "|" + String(current_battery, 1) + "|0|0";
  LoRa.beginPacket();
  LoRa.print(hello);
  LoRa.endPacket();
  current_battery -= TX_COST;
  clamp_battery();

  Serial.println("┌─────────────────────────────────────");
  Serial.println("│ [INIT] Ready to send: \"" + message + "\"");
  Serial.println("│ Step 2: Probe HELLO sent to " + dest);
  Serial.println("│ Waiting up to " + String(HELLO_REPLY_WAIT) + "ms for reply...");
  Serial.println("└─────────────────────────────────────");
}

// ═══════════════════════════════════════════════════════
//  FIRE DATA — greedy selected, queue packet
// ═══════════════════════════════════════════════════════

void fire_pending_data() {
  waiting_for_reply = false;

  int idx = select_best_neighbor("");
  if (idx == -1) {
    Serial.println("[SEND] No valid relay — message dropped");
    pending_msg = ""; pending_dest = "";
    return;
  }

  String msg_id = String(random(10000, 99999));
  String packet = "DATA|" + String(NODE_ID) + "|" + pending_dest + "|" +
                  msg_id + "|0|" + String(current_battery, 1) +
                  "|" + pending_msg + "|" + String(NODE_ID);
  enqueue_tx(packet);
  add_to_cache(msg_id);

  Serial.println("┌─────────────────────────────────────");
  Serial.println("│ [SEND] Queued for TX");
  Serial.println("│ To          : Node " + pending_dest);
  Serial.println("│ Msg         : \"" + pending_msg + "\"");
  Serial.println("│ Via         : Node " + neighbor_table[idx].id);
  Serial.println("│ Greedy Score: " + String(neighbor_table[idx].score, 4));
  Serial.println("└─────────────────────────────────────");

  pending_msg = ""; pending_dest = "";
}

// ═══════════════════════════════════════════════════════
//  HANDLE INCOMING PACKET
// ═══════════════════════════════════════════════════════

void handle_packet(String raw, int rssi) {
  String f[9]; int fi = 0; String tok = "";
  for (int i = 0; i < (int)raw.length(); i++) {
    if (raw[i] == '|') { if (fi < 8) { f[fi++] = tok; tok = ""; } }
    else tok += raw[i];
  }
  f[fi] = tok;

  String type = f[0];

  // ── HELLO: format HELLO|src|battery|hop|is_reply ──────────
  if (type == "HELLO") {
    String src       = f[1];
    float  batt      = f[2].toFloat();
    int    hop       = f[3].toInt();
    bool   is_reply  = (f[4] == "1");

    update_neighbor(src, rssi, batt, hop);
    float s = compute_score(rssi, batt);
    Serial.println("[RX] HELLO from Node " + src +
                   (is_reply ? " (reply)" : " (probe)") +
                   " | RSSI:" + rssi +
                   " | Batt:" + String(batt, 1) +
                   " | Score:" + String(s, 4));

    // Only auto-reply to genuine probes, never to a reply —
    // otherwise nodes ping-pong HELLOs forever.
    if (!is_reply) {
      String reply = "HELLO|" + String(NODE_ID) + "|" + String(current_battery, 1) + "|0|1";
      LoRa.beginPacket();
      LoRa.print(reply);
      LoRa.endPacket();
      current_battery -= TX_COST;
      clamp_battery();
      Serial.println("[RX] HELLO reply sent to Node " + src);
    }

    // If WE sent a probe and this is the reply we were waiting for
    if (waiting_for_reply && src == pending_dest && is_reply) {
      Serial.println("[PROBE] Reply from Node " + src + " — running greedy...");
      fire_pending_data();
    }
    return;
  }

  // ── DATA: format DATA|src|dest|mid|hop|batt|msg|route ─────
  String src   = f[1];
  String dest  = f[2];
  String mid   = f[3];
  int    hop   = f[4].toInt();
  float  batt  = f[5].toFloat();
  String data  = f[6];
  String route = f[7];

  if (in_cache(mid)) return;
  add_to_cache(mid);
  update_neighbor(src, rssi, batt, hop);

  if (dest == String(NODE_ID)) {
    float score = compute_score(rssi, batt);
    Serial.println("╔══════════════════════════════════════╗");
    Serial.println("║  [RX] MESSAGE RECEIVED AT NODE " + String(NODE_ID) + "     ║");
    Serial.println("╠══════════════════════════════════════╣");
    Serial.println("║  From        : Node " + src);
    Serial.println("║  Msg         : " + data);
    Serial.println("║  Greedy Score: " + String(score, 4));
    Serial.println("║  Route       : " + route + "→" + String(NODE_ID));
    Serial.println("║  RSSI        : " + String(rssi) + " dBm");
    Serial.println("║  Hops        : " + String(hop));
    Serial.println("╚══════════════════════════════════════╝");
    return;
  }

  if (hop >= MAX_HOPS)  { Serial.println("[DROP] Max hops reached"); return; }
  if (current_battery < LOW_BATTERY_THRESHOLD) { Serial.println("[DROP] Low battery"); return; }

  // Forward
  int idx = select_best_neighbor(src);
  if (idx == -1) { Serial.println("[FWD] No relay available"); return; }

  String fw[8]; int fwi = 0; String fwtok = "";
  for (int i = 0; i < (int)raw.length() && fwi < 7; i++) {
    if (raw[i] == '|') { fw[fwi++] = fwtok; fwtok = ""; } else fwtok += raw[i];
  }
  fw[fwi] = fwtok;
  fw[4] = String(hop + 1);
  fw[7] = fw[7] + "→" + String(NODE_ID);
  enqueue_tx(fw[0]+"|"+fw[1]+"|"+fw[2]+"|"+fw[3]+"|"+
             fw[4]+"|"+fw[5]+"|"+fw[6]+"|"+fw[7]);
  Serial.println("[FWD] Forwarding via Node " + neighbor_table[idx].id);
}

// ═══════════════════════════════════════════════════════
//  READ SERIAL — non-blocking
//  Format: <DEST_ID>:<message>   e.g.  "A:hello there"
// ═══════════════════════════════════════════════════════

void read_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      serial_buf.trim();
      if (serial_buf.length() > 0) {
        if (waiting_for_reply) {
          Serial.println("[BUSY] Still waiting for HELLO reply, please wait...");
        } else {
          int sep = serial_buf.indexOf(':');
          if (sep == -1) {
            Serial.println("[ERR] Format: <DEST_ID>:<message>   e.g. A:hello");
          } else {
            String dest = serial_buf.substring(0, sep);
            String msg  = serial_buf.substring(sep + 1);
            dest.trim();
            if (dest.length() == 0) {
              Serial.println("[ERR] No destination given. Format: <DEST_ID>:<message>");
            } else if (dest == String(NODE_ID)) {
              Serial.println("[ERR] Can't send to self");
            } else {
              Serial.println("┌─────────────────────────────────────");
              Serial.println("│ [INIT] Ready");
              Serial.println("│ Message: \"" + msg + "\"");
              Serial.println("│ Step 1: Sending probe HELLO to " + dest + "...");
              Serial.println("└─────────────────────────────────────");
              probe_and_send(dest, msg);
            }
          }
        }
        serial_buf = "";
      }
    } else {
      serial_buf += c;
    }
  }
}

// ═══════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);
  randomSeed(analogRead(A0) + millis());

  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   LoRa Mesh Node " + String(NODE_ID) + "  (ESP8266)        ║");
  Serial.println("║   IDLE = pure RX, no background TX   ║");
  Serial.println("║   Type <DEST_ID>:<message> + Enter   ║");
  Serial.println("╚══════════════════════════════════════╝");

  for (int i = 0; i < TX_QUEUE_SIZE; i++) tx_queue[i].valid = false;

  pinMode(NSS, OUTPUT);
  digitalWrite(NSS, HIGH);
  pinMode(RST, OUTPUT);
  digitalWrite(RST, LOW);  delay(200);
  digitalWrite(RST, HIGH); delay(200);

  SPI.begin();
  LoRa.setPins(NSS, RST, DIO0);

  if (!LoRa.begin(FREQUENCY)) { Serial.println("LoRa FAILED!"); while (1); }

  LoRa.setTxPower(17);
  LoRa.setSpreadingFactor(SPREADING_FACTOR);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);
  LoRa.enableCrc();

  current_battery   = INITIAL_BATTERY;
  last_drain_time   = millis();
  backoff_until     = 0;
  waiting_for_reply = false;
  neighbor_count    = 0;
  cache_size        = 0;
  serial_buf        = "";

  Serial.println("[INIT] Ready | Node ID: " + String(NODE_ID) + " | Battery: " + String(current_battery, 1) + "%");
  Serial.println("[INIT] Idle — type <DEST_ID>:<message> and press Enter to send");
}

// ═══════════════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════════════

void loop() {

  // 1. Idle battery drain — once per second
  if (millis() - last_drain_time >= 1000) {
    current_battery -= IDLE_DRAIN;
    clamp_battery();
    last_drain_time = millis();
  }

  // 2. RX — always listening
  int pkt = LoRa.parsePacket();
  if (pkt > 0) {
    String raw = "";
    while (LoRa.available()) raw += (char)LoRa.read();
    int rssi = LoRa.packetRssi();
    current_battery -= RX_COST;
    clamp_battery();
    handle_packet(raw, rssi);
  }

  // 3. Probe timeout — destination never replied, use cached table
  if (waiting_for_reply &&
      (millis() - hello_sent_at) >= HELLO_REPLY_WAIT) {
    Serial.println("[PROBE] Timeout — " + pending_dest + " did not reply, using cached table");
    fire_pending_data();
  }

  // 4. Read serial input
  read_serial();

  // 5. Drain TX queue
  process_tx_queue();
}
