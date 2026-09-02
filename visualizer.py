"""
Scalable LoRa Multi-Hop Network Simulator
==========================================

This is a scaled-up rewrite of the original 5-node demo, supporting anywhere
from 10 to 10,000+ nodes with a responsive GUI, CSMA/CA collision modeling,
an energy model, adaptive visualization, and statistics/CSV logging.

IMPORTANT: The following three functions are UNCHANGED from the original
program. Their logic (score calculation, routing/node-selection behavior,
RSSI formula) has not been touched in any way. Everything else in this file
is new supporting infrastructure (GUI, network generation, energy, CSMA,
stats, visualization) built around them.

    - compute_rssi()
    - greedy_neighbor()
    - custom_greedy()
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import math
import time
import random
import csv
import os

try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


# =====================================================================
# LoRa PHY MODEL  --  UNCHANGED FROM ORIGINAL, DO NOT MODIFY
# =====================================================================
SF_SENS = {7: -123, 8: -126, 9: -129, 10: -132, 11: -134, 12: -137}


def compute_rssi(d, Pt=14, freq=433e6, n=3.0):
    if d < 1:
        d = 1
    PL0 = 32.44 + 20 * math.log10(freq / 1e6)
    PL = PL0 + 10 * n * math.log10(d)
    return Pt - PL


ALGO_GREEDY_NEIGHBOR = "Greedy Neighbor Routing"
ALGO_CUSTOM = "Custom Greedy Algorithm"


# =====================================================================
# ROUTING ALGORITHMS  --  UNCHANGED FROM ORIGINAL, DO NOT MODIFY
# (score calculations, node selection logic, and path selection
# behavior are byte-for-byte identical to the original program)
# =====================================================================
def greedy_neighbor(src, neighbors, rssi, visited):
    scores = {}
    for n in neighbors[src]:
        if n not in visited:
            scores[n] = rssi[(src, n)]
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores


def custom_greedy(src, neighbors, rssi, battery, visited, w1=0.6, w2=0.4):
    scores = {}
    for n in neighbors[src]:
        if n not in visited:
            score = w1 * rssi[(src, n)] + w2 * battery[n]
            scores[n] = score
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores


# =====================================================================
# NETWORK GENERATION  (new)
# Fast, vectorised position generation + KD-tree edge building so the
# program stays responsive from 10 up to 10,000+ nodes. (Plain
# nx.spring_layout / nx.random_layout style force-directed layouts are
# O(n^2) or worse per iteration and become unusable well before 10,000
# nodes, so equivalent-purpose numpy/KD-tree based deployment patterns
# are used here instead -- random, grid, circular(ring) and cluster.)
# =====================================================================
class NetworkGenerator:

    @staticmethod
    def generate_positions(num_nodes, mode="random", area_size=1000.0,
                            num_clusters=6, cluster_std=40.0, seed=None):
        rng = np.random.default_rng(seed)

        if mode == "grid":
            side = int(math.ceil(math.sqrt(num_nodes)))
            spacing = area_size / max(side - 1, 1)
            xs, ys = np.meshgrid(np.arange(side), np.arange(side))
            pts = np.column_stack([xs.ravel(), ys.ravel()]) * spacing
            pos = pts[:num_nodes].astype(float)

        elif mode == "circular":
            radius = area_size / 2.0
            rings = max(1, num_nodes // 200 + 1)
            per_ring = math.ceil(num_nodes / rings)
            pos = np.zeros((num_nodes, 2))
            idx = 0
            for r_i in range(rings):
                r = radius * (r_i + 1) / rings
                count = min(per_ring, num_nodes - idx)
                if count <= 0:
                    break
                ang = np.linspace(0, 2 * math.pi, count, endpoint=False)
                pos[idx:idx + count, 0] = radius + r * np.cos(ang)
                pos[idx:idx + count, 1] = radius + r * np.sin(ang)
                idx += count

        elif mode == "cluster":
            centers = rng.uniform(area_size * 0.1, area_size * 0.9, size=(num_clusters, 2))
            assign = rng.integers(0, num_clusters, size=num_nodes)
            pos = centers[assign] + rng.normal(0, cluster_std, size=(num_nodes, 2))
            pos = np.clip(pos, 0, area_size)

        else:  # "random"
            pos = rng.uniform(0, area_size, size=(num_nodes, 2))

        return pos

    @staticmethod
    def max_range(sf, Pt=14, freq=433e6, n=3.0):
        """
        Inverts compute_rssi(d) == SF_SENS[sf] to find the max distance at
        which a link is still viable, WITHOUT changing compute_rssi itself.
        """
        sens = SF_SENS[sf]
        PL0 = 32.44 + 20 * math.log10(freq / 1e6)
        exponent = (Pt - sens - PL0) / (10 * n)
        return 10 ** exponent

    @staticmethod
    def build_edges(positions, max_range, edge_cap=300000):
        """
        Builds all node pairs within max_range using a KD-tree so this stays
        fast at 10,000 nodes (O(n log n) instead of O(n^2)). If the resulting
        edge count is extreme (very dense deployment) it is randomly capped
        for responsiveness, and the caller is told so it can warn the user.
        """
        n = len(positions)
        capped = False

        if HAVE_SCIPY:
            tree = cKDTree(positions)
            pairs = tree.query_pairs(r=max_range)
            edges = list(pairs)
        else:
            edges = []
            for i in range(n):
                for j in range(i + 1, n):
                    d = np.linalg.norm(positions[i] - positions[j])
                    if d <= max_range:
                        edges.append((i, j))

        if len(edges) > edge_cap:
            edges = random.sample(edges, edge_cap)
            capped = True

        return edges, capped


# =====================================================================
# ENERGY MODEL  (new)
# Transmission, reception, idle listening and sleep all consume energy.
# Battery values it updates are the SAME dict/array read by the unchanged
# routing functions above, so routing continues to react to real battery
# levels exactly as before -- only how battery gets depleted is new.
# =====================================================================
class EnergyModel:
    def __init__(self, num_nodes, tx_cost=2.0, rx_cost=1.2, idle_cost=0.05, sleep_cost=0.01):
        self.tx_cost = tx_cost
        self.rx_cost = rx_cost
        self.idle_cost = idle_cost
        self.sleep_cost = sleep_cost
        self.consumed = np.zeros(num_nodes)

    def _apply(self, battery, node, cost):
        battery[node] = max(0.0, battery[node] - cost)
        self.consumed[node] += cost
        return cost

    def apply_tx(self, battery, node):
        return self._apply(battery, node, self.tx_cost)

    def apply_rx(self, battery, node):
        return self._apply(battery, node, self.rx_cost)

    def apply_idle(self, battery, node):
        return self._apply(battery, node, self.idle_cost)

    def apply_sleep(self, battery, node):
        return self._apply(battery, node, self.sleep_cost)


# =====================================================================
# CSMA/CA COLLISION SIMULATOR  (new)
# =====================================================================
class CSMASimulator:
    """
    Simplified CSMA/CA model: channel sensing, exponential backoff,
    collision probability that scales with contending neighbors, retries
    up to max_retries, then a drop. Produces the statistics requested:
    collision rate, successful/dropped packets, retransmissions, channel
    utilization, throughput and packet delivery ratio.
    """

    def __init__(self, energy_model, slot_time=0.01, max_retries=5,
                 backoff_min=1, backoff_max=8):
        self.energy = energy_model
        self.slot_time = slot_time
        self.max_retries = max_retries
        self.backoff_min = backoff_min
        self.backoff_max = backoff_max
        self.reset_stats()

    def reset_stats(self):
        self.stats = {
            "attempts": 0, "successful": 0, "collisions": 0,
            "dropped": 0, "retransmissions": 0, "busy_channel": 0,
            "history_success": [], "history_collision": [], "history_retry": [],
        }

    def simulate_hop(self, sender, receiver, battery, rng, contenders=None):
        retries = 0
        success = False
        contenders = contenders if contenders is not None else int(rng.integers(0, 4))

        while retries <= self.max_retries:
            self.stats["attempts"] += 1

            # --- Channel sensing ---
            channel_busy = rng.random() < min(0.5, 0.05 * contenders)
            if channel_busy:
                self.stats["busy_channel"] += 1
                self.energy.apply_idle(battery, sender)
                retries += 1
                self.stats["retransmissions"] += 1
                self.stats["history_retry"].append(1)
                continue

            # --- Transmission attempt / collision check ---
            collision = rng.random() < min(0.9, 0.08 * contenders)
            self.energy.apply_tx(battery, sender)
            self.energy.apply_rx(battery, receiver)

            if collision:
                self.stats["collisions"] += 1
                self.stats["history_collision"].append(1)
                self.stats["history_success"].append(0)
                retries += 1
                self.stats["retransmissions"] += 1
                continue
            else:
                success = True
                self.stats["successful"] += 1
                self.stats["history_success"].append(1)
                self.stats["history_collision"].append(0)
                break

        if not success:
            self.stats["dropped"] += 1

        return success

    def summary(self):
        attempts = max(1, self.stats["attempts"])
        pdr_den = max(1, self.stats["successful"] + self.stats["dropped"])
        return {
            "Successful Packets": self.stats["successful"],
            "Dropped Packets": self.stats["dropped"],
            "Collisions": self.stats["collisions"],
            "Retransmissions": self.stats["retransmissions"],
            "Collision Rate": round(self.stats["collisions"] / attempts, 4),
            "Channel Utilization": round(self.stats["busy_channel"] / attempts, 4),
            "Packet Delivery Ratio": round(self.stats["successful"] / pdr_den, 4),
            "Throughput (pkts/s)": round(self.stats["successful"] / max(attempts * self.slot_time, 1e-6), 3),
        }


# =====================================================================
# STATISTICS / CSV LOGGING  (new)
# =====================================================================
class StatisticsLogger:
    def __init__(self):
        self.routing_log = []
        self.energy_log = []
        self.battery_log = []

    def log_routing(self, hop_from, hop_to, score, algo):
        self.routing_log.append({
            "from": hop_from, "to": hop_to, "score": round(float(score), 4),
            "algo": algo, "time": time.time(),
        })

    def log_energy(self, node, event, cost, remaining_battery):
        self.energy_log.append({
            "node": node, "event": event, "cost": cost,
            "remaining_battery": round(float(remaining_battery), 3), "time": time.time(),
        })

    def export_csv(self, folder):
        os.makedirs(folder, exist_ok=True)
        self._write_csv(os.path.join(folder, "routing_log.csv"), self.routing_log)
        self._write_csv(os.path.join(folder, "energy_log.csv"), self.energy_log)

    @staticmethod
    def _write_csv(path, rows):
        if not rows:
            return
        keys = rows[0].keys()
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(rows)


# =====================================================================
# VISUALIZATION  (new)
# Adaptive rendering: full labeled draw under 100 nodes, small unlabeled
# markers 100-500, and a lightweight density scatter above 500 so the
# GUI stays responsive at thousands of nodes. Pan/zoom comes for free
# from matplotlib's NavigationToolbar2Tk, wired in by the Simulator.
# =====================================================================
class VisualizationManager:
    def __init__(self, ax):
        self.ax = ax

    def draw(self, positions, edges, battery, path=None, num_nodes=0):
        self.ax.clear()
        if num_nodes <= 100:
            self._draw_full(positions, edges, battery, path)
        elif num_nodes <= 500:
            self._draw_medium(positions, edges, path)
        else:
            self._draw_density(positions, path)
        self.ax.set_title(f"Network View ({num_nodes} nodes, {len(edges)} links)")
        self.ax.set_aspect("equal", adjustable="datalim")

    def _draw_full(self, positions, edges, battery, path):
        xs, ys = positions[:, 0], positions[:, 1]
        for (u, v) in edges:
            self.ax.plot([xs[u], xs[v]], [ys[u], ys[v]], color="lightgray", linewidth=0.6, zorder=1)
        self.ax.scatter(xs, ys, s=250, c="lightblue", edgecolors="black", zorder=2)
        for i in range(len(positions)):
            self.ax.annotate(f"{i}\n{battery[i]:.0f}%", (xs[i], ys[i]), ha="center", va="center",
                              fontsize=7, zorder=3)
        if path:
            self._draw_path(positions, path)

    def _draw_medium(self, positions, edges, path):
        xs, ys = positions[:, 0], positions[:, 1]
        for (u, v) in edges:
            self.ax.plot([xs[u], xs[v]], [ys[u], ys[v]], color="lightgray", linewidth=0.3, zorder=1)
        self.ax.scatter(xs, ys, s=25, c="lightblue", edgecolors="none", zorder=2)
        if path:
            self._draw_path(positions, path)

    def _draw_density(self, positions, path):
        xs, ys = positions[:, 0], positions[:, 1]
        self.ax.scatter(xs, ys, s=2, c="steelblue", alpha=0.5, zorder=2)
        if path:
            self._draw_path(positions, path, linewidth=2.0, markersize=6)

    def _draw_path(self, positions, path, linewidth=2.5, markersize=9):
        px = [positions[n][0] for n in path]
        py = [positions[n][1] for n in path]
        self.ax.plot(px, py, color="red", linewidth=linewidth, zorder=4)
        self.ax.scatter(px, py, s=markersize ** 2, c="orange", edgecolors="red", zorder=5)


# =====================================================================
# PARAMETER INPUT WINDOW  (new -- scalable, no per-node widgets)
# =====================================================================
class ParamInput:
    def __init__(self, root, algo):
        self.top = tk.Toplevel(root)
        self.top.title("Simulation Parameters")
        self.top.geometry("600x680")
        self.algo = algo
        self.result = None
        self.battery = None  # numpy array, created lazily

        pad = {"padx": 6, "pady": 4}

        # ---------------- Network generation ----------------
        frame_net = tk.LabelFrame(self.top, text="Network Generation")
        frame_net.pack(fill="x", **pad)

        tk.Label(frame_net, text="Number of Nodes (10 - 10000):").grid(row=0, column=0, sticky="w")
        self.num_nodes_var = tk.IntVar(value=20)
        tk.Spinbox(frame_net, from_=10, to=10000, textvariable=self.num_nodes_var, width=10).grid(row=0, column=1)

        tk.Label(frame_net, text="Deployment:").grid(row=1, column=0, sticky="w")
        self.deploy_var = tk.StringVar(value="random")
        tk.OptionMenu(frame_net, self.deploy_var, "random", "grid", "circular", "cluster").grid(row=1, column=1, sticky="w")

        tk.Label(frame_net, text="Area Size (m):").grid(row=2, column=0, sticky="w")
        self.area_var = tk.DoubleVar(value=1000.0)
        tk.Entry(frame_net, textvariable=self.area_var, width=10).grid(row=2, column=1, sticky="w")

        tk.Label(frame_net, text="Spreading Factor (SF):").grid(row=3, column=0, sticky="w")
        self.sf_var = tk.IntVar(value=9)
        tk.OptionMenu(frame_net, self.sf_var, *SF_SENS.keys()).grid(row=3, column=1, sticky="w")

        self.auto_range_var = tk.BooleanVar(value=False)
        tk.Checkbutton(frame_net, text="Auto transmission range (derived from SF sensitivity)",
                        variable=self.auto_range_var).grid(row=4, column=0, columnspan=2, sticky="w")
        tk.Label(frame_net, text="Manual Range (m, used if auto is off):").grid(row=5, column=0, sticky="w")
        self.range_var = tk.DoubleVar(value=150.0)
        tk.Entry(frame_net, textvariable=self.range_var, width=10).grid(row=5, column=1, sticky="w")

        tk.Label(frame_net, text="Tip: for large node counts, increase Area Size or\n"
                                  "reduce Range to keep link density (and runtime) low.",
                 fg="gray", justify="left").grid(row=6, column=0, columnspan=2, sticky="w")

        # ---------------- Battery bulk assignment ----------------
        frame_batt = tk.LabelFrame(self.top, text="Battery Assignment (%)")
        frame_batt.pack(fill="x", **pad)

        tk.Label(frame_batt, text="Default Battery:").grid(row=0, column=0, sticky="w")
        self.default_batt_var = tk.DoubleVar(value=80.0)
        tk.Entry(frame_batt, textvariable=self.default_batt_var, width=8).grid(row=0, column=1)
        tk.Button(frame_batt, text="Apply to All Nodes", command=self.apply_default_battery).grid(row=0, column=2, columnspan=2, sticky="ew")

        tk.Label(frame_batt, text="Range:").grid(row=1, column=0, sticky="w")
        tk.Label(frame_batt, text="Start").grid(row=1, column=1)
        self.batt_start_var = tk.IntVar(value=0)
        tk.Entry(frame_batt, textvariable=self.batt_start_var, width=6).grid(row=2, column=1)
        tk.Label(frame_batt, text="End").grid(row=1, column=2)
        self.batt_end_var = tk.IntVar(value=0)
        tk.Entry(frame_batt, textvariable=self.batt_end_var, width=6).grid(row=2, column=2)
        tk.Label(frame_batt, text="Battery %").grid(row=1, column=3)
        self.batt_range_val_var = tk.DoubleVar(value=50.0)
        tk.Entry(frame_batt, textvariable=self.batt_range_val_var, width=6).grid(row=2, column=3)
        tk.Button(frame_batt, text="Apply Range", command=self.apply_range_battery).grid(row=2, column=4, sticky="ew")

        tk.Label(frame_batt, text="Random Battery Min/Max:").grid(row=3, column=0, sticky="w")
        self.rand_batt_min = tk.DoubleVar(value=20.0)
        self.rand_batt_max = tk.DoubleVar(value=100.0)
        tk.Entry(frame_batt, textvariable=self.rand_batt_min, width=6).grid(row=3, column=1)
        tk.Entry(frame_batt, textvariable=self.rand_batt_max, width=6).grid(row=3, column=2)
        tk.Button(frame_batt, text="Random Batteries", command=self.apply_random_battery).grid(row=3, column=3, columnspan=2, sticky="ew")

        # ---------------- Distance / deployment jitter ----------------
        frame_dist = tk.LabelFrame(self.top, text="Link Distance Controls")
        frame_dist.pack(fill="x", **pad)
        tk.Label(frame_dist,
                 text=("Link distances come from generated node positions\n"
                       "(not thousands of manual entries). Use these controls\n"
                       "to randomize the deployment instead of per-edge values."),
                 justify="left").grid(row=0, column=0, columnspan=4, sticky="w")

        tk.Label(frame_dist, text="Distance Jitter Min/Max (m):").grid(row=1, column=0, sticky="w")
        self.jitter_min = tk.DoubleVar(value=0.0)
        self.jitter_max = tk.DoubleVar(value=0.0)
        tk.Entry(frame_dist, textvariable=self.jitter_min, width=6).grid(row=1, column=1)
        tk.Entry(frame_dist, textvariable=self.jitter_max, width=6).grid(row=1, column=2)
        tk.Button(frame_dist, text="Random Distances (jitter positions)",
                  command=lambda: self.status_label.config(
                      text="Jitter will be applied to node positions on generation.")
                  ).grid(row=1, column=3, sticky="ew")

        # ---------------- Source / Destination ----------------
        frame_sd = tk.LabelFrame(self.top, text="Source / Destination")
        frame_sd.pack(fill="x", **pad)
        tk.Label(frame_sd, text="Source Node ID:").grid(row=0, column=0, sticky="w")
        self.src_var = tk.IntVar(value=0)
        tk.Entry(frame_sd, textvariable=self.src_var, width=8).grid(row=0, column=1)
        tk.Label(frame_sd, text="Destination Node ID:").grid(row=0, column=2, sticky="w")
        self.dst_var = tk.IntVar(value=1)
        tk.Entry(frame_sd, textvariable=self.dst_var, width=8).grid(row=0, column=3)
        tk.Button(frame_sd, text="Randomize Src/Dst", command=self.randomize_endpoints).grid(row=0, column=4, sticky="ew")

        # ---------------- Status + Start ----------------
        self.status_label = tk.Label(self.top, text="", fg="blue", justify="left", wraplength=560)
        self.status_label.pack(fill="x", **pad)

        tk.Button(self.top, text="Generate Network & Start", command=self.submit,
                  bg="#4CAF50", fg="white").pack(pady=10)

    # ---- battery bulk operations ----
    def _ensure_battery(self):
        n = self.num_nodes_var.get()
        if self.battery is None or len(self.battery) != n:
            self.battery = np.full(n, self.default_batt_var.get(), dtype=float)

    def apply_default_battery(self):
        self._ensure_battery()
        self.battery[:] = self.default_batt_var.get()
        self.status_label.config(text=f"Applied {self.default_batt_var.get()}% battery to all {len(self.battery)} nodes.")

    def apply_range_battery(self):
        self._ensure_battery()
        s, e = self.batt_start_var.get(), self.batt_end_var.get()
        s = max(0, s)
        e = min(len(self.battery) - 1, e)
        if s > e:
            self.status_label.config(text="Invalid range: start must be <= end.")
            return
        self.battery[s:e + 1] = self.batt_range_val_var.get()
        self.status_label.config(text=f"Applied {self.batt_range_val_var.get()}% to nodes {s}-{e}.")

    def apply_random_battery(self):
        self._ensure_battery()
        lo, hi = self.rand_batt_min.get(), self.rand_batt_max.get()
        self.battery[:] = np.random.uniform(lo, hi, size=len(self.battery))
        self.status_label.config(text=f"Randomized battery in range [{lo}, {hi}]%.")

    def randomize_endpoints(self):
        n = self.num_nodes_var.get()
        if n >= 2:
            s, d = random.sample(range(n), 2)
            self.src_var.set(s)
            self.dst_var.set(d)

    def submit(self):
        n = self.num_nodes_var.get()
        if n < 2:
            messagebox.showerror("Invalid input", "Need at least 2 nodes.")
            return
        self._ensure_battery()
        src, dst = self.src_var.get(), self.dst_var.get()
        if not (0 <= src < n and 0 <= dst < n):
            messagebox.showerror("Invalid input", "Source/Destination must be valid node IDs.")
            return
        if src == dst:
            messagebox.showerror("Invalid input", "Source and destination must differ.")
            return

        self.result = {
            "num_nodes": n,
            "deployment": self.deploy_var.get(),
            "area_size": self.area_var.get(),
            "sf": self.sf_var.get(),
            "auto_range": self.auto_range_var.get(),
            "manual_range": self.range_var.get(),
            "battery": self.battery.copy(),
            "jitter": (self.jitter_min.get(), self.jitter_max.get()),
            "source": src,
            "destination": dst,
        }
        self.top.destroy()


# =====================================================================
# MAIN SIMULATOR WINDOW  (new, wraps the unchanged routing algorithms)
# =====================================================================
class Simulator:
    def __init__(self, root, algo, params):
        self.root = root
        self.algo = algo
        self.params = params
        self.num_nodes = params["num_nodes"]
        self.battery = params["battery"]
        self.sf = params["sf"]
        self.src = params["source"]
        self.dst = params["destination"]

        self.root.title("LoRa Multi-Hop Network Simulator")
        self.root.geometry("1180x780")

        self.logger = StatisticsLogger()
        self.energy = EnergyModel(self.num_nodes)
        self.csma = CSMASimulator(self.energy)
        self.rng = np.random.default_rng()

        self._build_network()
        self._build_gui()

        self.running = False
        self.paused = False
        self.speed = 0.5
        self.path = [self.src]
        self.visited = set([self.src])
        self.current = self.src
        self.finished = False

        self.draw()

    # ---------------- network construction ----------------
    def _build_network(self):
        p = self.params
        self.positions = NetworkGenerator.generate_positions(
            self.num_nodes, mode=p["deployment"], area_size=p["area_size"])

        jitter_min, jitter_max = p["jitter"]
        if jitter_max > 0:
            mags = self.rng.uniform(jitter_min, jitter_max, size=self.positions.shape)
            signs = self.rng.choice([-1, 1], size=self.positions.shape)
            self.positions = self.positions + mags * signs

        self.max_range = (NetworkGenerator.max_range(self.sf) if p["auto_range"]
                           else p["manual_range"])

        edge_pairs, capped = NetworkGenerator.build_edges(self.positions, self.max_range)
        if capped:
            messagebox.showwarning(
                "Large network",
                "Link density was very high for the chosen area/range, so the edge "
                "set was capped at 300,000 links for responsiveness. Increase Area "
                "Size or reduce Range for a more realistic sparse topology.")

        self.neighbors = {n: set() for n in range(self.num_nodes)}
        self.rssi = {}
        self.edges = []
        for (u, v) in edge_pairs:
            d = float(np.linalg.norm(self.positions[u] - self.positions[v]))
            r = compute_rssi(d)  # UNCHANGED function
            self.rssi[(u, v)] = self.rssi[(v, u)] = r
            self.neighbors[u].add(v)
            self.neighbors[v].add(u)
            self.edges.append((u, v))

    # ---------------- gui construction ----------------
    def _build_gui(self):
        main = tk.Frame(self.root)
        main.pack(fill="both", expand=True)

        left = tk.Frame(main)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(main, width=380)
        right.pack(side="right", fill="y")

        self.fig, self.ax = plt.subplots(figsize=(6.5, 6.5))
        self.canvas = FigureCanvasTkAgg(self.fig, left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, left)
        toolbar.update()

        self.viz = VisualizationManager(self.ax)

        ctrl = tk.LabelFrame(right, text="Simulation Controls")
        ctrl.pack(fill="x", padx=6, pady=6)
        tk.Button(ctrl, text="Start", command=self.start).grid(row=0, column=0, sticky="ew")
        tk.Button(ctrl, text="Pause", command=self.pause).grid(row=0, column=1, sticky="ew")
        tk.Button(ctrl, text="Resume", command=self.resume).grid(row=0, column=2, sticky="ew")
        tk.Button(ctrl, text="Reset", command=self.reset).grid(row=0, column=3, sticky="ew")
        tk.Button(ctrl, text="Step", command=self.step).grid(row=1, column=0, sticky="ew")
        tk.Button(ctrl, text="Export CSV", command=self.export_csv).grid(row=1, column=1, columnspan=3, sticky="ew")

        tk.Label(ctrl, text="Speed (s/hop):").grid(row=2, column=0, columnspan=2, sticky="w")
        self.speed_var = tk.DoubleVar(value=0.5)
        tk.Scale(ctrl, from_=0.05, to=2.0, resolution=0.05, orient="horizontal",
                 variable=self.speed_var, command=self._on_speed_change).grid(row=2, column=2, columnspan=2, sticky="ew")

        stats_frame = tk.LabelFrame(right, text="Routing / Stats Log")
        stats_frame.pack(fill="both", expand=True, padx=6, pady=6)
        self.score_box = tk.Text(stats_frame, height=20, width=46)
        self.score_box.pack(fill="both", expand=True)

        graphs = tk.LabelFrame(right, text="Graphs")
        graphs.pack(fill="x", padx=6, pady=6)
        tk.Button(graphs, text="Energy Graphs", command=self.show_energy_graphs).grid(row=0, column=0, sticky="ew")
        tk.Button(graphs, text="Collision Graphs", command=self.show_collision_graphs).grid(row=0, column=1, sticky="ew")
        tk.Button(graphs, text="Routing Stats", command=self.show_routing_graphs).grid(row=1, column=0, columnspan=2, sticky="ew")

        info = f"Nodes: {self.num_nodes} | Edges: {len(self.edges)} | Max Range: {self.max_range:.1f} m"
        tk.Label(right, text=info, fg="gray").pack(anchor="w", padx=6)

    def _on_speed_change(self, val):
        self.speed = float(val)

    # ---------------- drawing ----------------
    def draw(self):
        self.viz.draw(self.positions, self.edges, self.battery, path=self.path, num_nodes=self.num_nodes)
        self.canvas.draw()

    # ---------------- simulation control ----------------
    def start(self):
        if self.finished:
            self.reset()
        self.running = True
        self.paused = False
        self.score_box.insert(tk.END, "\n--- Simulation Started ---\n")
        self._loop()

    def pause(self):
        self.paused = True

    def resume(self):
        if self.running:
            self.paused = False
            self._loop()

    def reset(self):
        self.running = False
        self.paused = False
        self.finished = False
        self.path = [self.src]
        self.visited = set([self.src])
        self.current = self.src
        self.battery = self.params["battery"].copy()
        self.energy = EnergyModel(self.num_nodes)
        self.csma.reset_stats()
        self.logger = StatisticsLogger()
        self.score_box.delete("1.0", tk.END)
        self.draw()

    def step(self):
        if self.finished:
            return
        self._advance_one_hop()
        self.draw()

    def _loop(self):
        if not self.running or self.paused or self.finished:
            return
        self._advance_one_hop()
        self.draw()
        if not self.finished:
            self.root.after(int(self.speed * 1000), self._loop)

    # ---------------- core hop logic ----------------
    def _advance_one_hop(self):
        if self.current == self.dst:
            self.finished = True
            self.score_box.insert(tk.END, "\nDestination reached successfully.\n")
            return

        # ROUTING DECISION -- uses the UNCHANGED algorithm functions
        if self.algo == ALGO_GREEDY_NEIGHBOR:
            nxt, scores = greedy_neighbor(self.current, self.neighbors, self.rssi, self.visited)
        else:
            nxt, scores = custom_greedy(self.current, self.neighbors, self.rssi, self.battery, self.visited)

        if not scores or nxt is None:
            self.score_box.insert(tk.END, f"\nNode {self.current} has no unvisited neighbors.\n")
            self.finished = True
            self.running = False
            return

        self.score_box.insert(tk.END, f"\nCurrent Node: {self.current}\n")
        shown = list(scores.items())[:12]
        for node, score in shown:
            self.score_box.insert(tk.END, f"Neighbor {node} -> Score = {round(score, 2)}\n")
        if len(scores) > 12:
            self.score_box.insert(tk.END, f"... and {len(scores) - 12} more neighbors\n")
        self.score_box.insert(tk.END, f"Selected Next Hop: {nxt}\n")
        self.score_box.see(tk.END)

        self.logger.log_routing(self.current, nxt, scores[nxt], self.algo)

        # CSMA/CA simulation for this hop
        contenders = min(len(self.neighbors[self.current]), 10)
        success = self.csma.simulate_hop(self.current, nxt, self.battery, self.rng, contenders=contenders)

        # Idle-listening energy cost for nearby nodes that overheard the slot
        for other in self.neighbors[self.current]:
            if other not in (self.current, nxt):
                self.energy.apply_idle(self.battery, other)

        self.logger.log_energy(self.current, "tx", self.energy.tx_cost, self.battery[self.current])
        self.logger.log_energy(nxt, "rx", self.energy.rx_cost, self.battery[nxt])

        if not success:
            self.score_box.insert(tk.END, f"Transmission to {nxt} failed after retries (dropped).\n")

        self.path.append(nxt)
        self.visited.add(nxt)
        self.current = nxt

        if self.current == self.dst:
            self.finished = True
            self.running = False
            self.score_box.insert(tk.END, "\nDestination reached successfully.\n")

    # ---------------- CSV export ----------------
    def export_csv(self):
        folder = filedialog.askdirectory(title="Choose export folder") or "."
        self.logger.export_csv(folder)
        messagebox.showinfo("Export complete", f"Logs exported to {folder}")

    # ---------------- graph windows ----------------
    def show_energy_graphs(self):
        win = tk.Toplevel(self.root)
        win.title("Energy Spectrum")
        fig, axes = plt.subplots(2, 2, figsize=(9, 7))
        limit = min(200, self.num_nodes)
        nodes = np.arange(limit)

        axes[0, 0].bar(nodes, self.battery[:limit], color="green")
        axes[0, 0].set_title(f"Remaining Battery vs Node (first {limit})")

        axes[0, 1].bar(nodes, self.energy.consumed[:limit], color="orange")
        axes[0, 1].set_title(f"Energy Consumed vs Node (first {limit})")

        axes[1, 0].hist(self.energy.consumed, bins=30, color="steelblue")
        axes[1, 0].set_title(f"Network Energy Histogram (avg={self.energy.consumed.mean():.2f})")

        sc = axes[1, 1].scatter(self.positions[:, 0], self.positions[:, 1], c=self.battery, cmap="RdYlGn", s=8)
        axes[1, 1].set_title("Battery Heat Map")
        fig.colorbar(sc, ax=axes[1, 1])

        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        canvas.draw()

    def show_collision_graphs(self):
        win = tk.Toplevel(self.root)
        win.title("Collision / CSMA Statistics")
        fig, axes = plt.subplots(2, 2, figsize=(9, 7))
        hist_s = self.csma.stats["history_success"]
        hist_c = self.csma.stats["history_collision"]

        axes[0, 0].plot(np.cumsum(hist_s) if hist_s else [0], color="green")
        axes[0, 0].set_title("Cumulative Successful Transmissions")

        axes[0, 1].plot(np.cumsum(hist_c) if hist_c else [0], color="red")
        axes[0, 1].set_title("Cumulative Collisions")

        axes[1, 0].plot(self.csma.stats["history_retry"] or [0], color="orange")
        axes[1, 0].set_title("Retries Over Time")

        summary = self.csma.summary()
        labels = list(summary.keys())
        values = [summary[k] for k in labels]
        axes[1, 1].barh(labels, values, color="steelblue")
        axes[1, 1].set_title("Summary Statistics")
        axes[1, 1].tick_params(axis="y", labelsize=7)

        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        canvas.draw()

    def show_routing_graphs(self):
        win = tk.Toplevel(self.root)
        win.title("Routing Statistics")
        fig, axes = plt.subplots(2, 2, figsize=(9, 7))

        hops = list(range(len(self.path)))
        axes[0, 0].plot(hops, self.path, marker="o")
        axes[0, 0].set_title("Path (Node ID per Hop)")
        axes[0, 0].set_xlabel("Hop Count")

        rssi_along = [self.rssi.get((self.path[i], self.path[i + 1]), 0) for i in range(len(self.path) - 1)]
        axes[0, 1].plot(rssi_along, marker="o", color="purple")
        axes[0, 1].set_title("RSSI Along Route")

        batt_along = [self.params["battery"][n] for n in self.path]
        axes[1, 0].plot(batt_along, marker="o", color="green")
        axes[1, 0].set_title("Battery Along Route (initial values)")

        scores = [r["score"] for r in self.logger.routing_log]
        axes[1, 1].plot(scores, marker="o", color="brown")
        axes[1, 1].set_title("Selected Node Scores")

        fig.tight_layout()
        canvas = FigureCanvasTkAgg(fig, win)
        canvas.get_tk_widget().pack(fill="both", expand=True)
        canvas.draw()


# =====================================================================
# MAIN FLOW
# =====================================================================
def main():
    root = tk.Tk()
    root.withdraw()

    algo = tk.StringVar(value=ALGO_CUSTOM)
    sel = tk.Toplevel(root)
    sel.title("Select Algorithm")
    for a in [ALGO_GREEDY_NEIGHBOR, ALGO_CUSTOM]:
        tk.Radiobutton(sel, text=a, variable=algo, value=a).pack(anchor="w")
    tk.Button(sel, text="Next", command=sel.destroy).pack()
    root.wait_window(sel)

    params_win = ParamInput(root, algo.get())
    root.wait_window(params_win.top)

    if params_win.result is None:
        root.destroy()
        return

    sim_win = tk.Toplevel(root)
    Simulator(sim_win, algo.get(), params_win.result)

    root.mainloop()


if __name__ == "__main__":
    main()
