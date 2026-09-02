# -*- coding: utf-8 -*-
"""
LoRa AI-Based Weight Optimizer with Monte Carlo Simulation
==========================================================

Architecture
------------
  PhyModel          – compute_rssi(), SF_SENS           [UNCHANGED from originals]
  RoutingAlgos      – greedy_neighbor(), custom_greedy() [UNCHANGED from originals]
  NetworkGenerator  – random / grid / circular / cluster positions, KD-tree edges
  EnergyModel       – TX / RX / idle depletion
  CSMASimulator     – CSMA/CA collision model + PDR
  MonteCarloEngine  – N manually-configurable trials per (w1,w2) candidate
                      across randomised area (10-1000 m), battery, nodes, deploy
  WeightOptimizer   – Coarse 10×10 grid → fine random search → best weights
  EmergencyProfiler – sklearn MLPRegressor trained on full MC dataset;
                      3 emergency profiles extracted via K-Means on results
  OptimizerApp      – 4-tab Tkinter GUI (Optimizer / Emergency / Results / Network)
"""

import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import numpy as np
import math
import random
import threading
import time
import csv
import os

# ── optional fast deps ──────────────────────────────────────────────
try:
    from scipy.spatial import cKDTree
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False

try:
    from sklearn.neural_network import MLPRegressor
    from sklearn.preprocessing import StandardScaler
    from sklearn.cluster import KMeans
    HAVE_SKLEARN = True
except ImportError:
    HAVE_SKLEARN = False


# =====================================================================
# LoRa PHY MODEL  ── UNCHANGED FROM ORIGINAL, DO NOT MODIFY
# =====================================================================
SF_SENS = {7: -123, 8: -126, 9: -129, 10: -132, 11: -134, 12: -137}


def compute_rssi(d, Pt=14, freq=433e6, n=3.0):
    """Log-distance path-loss model – identical to original."""
    if d < 1:
        d = 1
    PL0 = 32.44 + 20 * math.log10(freq / 1e6)
    PL  = PL0 + 10 * n * math.log10(d)
    return Pt - PL


# =====================================================================
# ROUTING ALGORITHMS  ── UNCHANGED FROM ORIGINAL, DO NOT MODIFY
# =====================================================================
def greedy_neighbor(src, neighbors, rssi, visited):
    """Selects next hop purely on RSSI – identical to original."""
    scores = {}
    for nb in neighbors[src]:
        if nb not in visited:
            scores[nb] = rssi[(src, nb)]
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores


def custom_greedy(src, neighbors, rssi, battery, visited, w1=0.6, w2=0.4):
    """Weighted RSSI + battery score – identical to original."""
    scores = {}
    for nb in neighbors[src]:
        if nb not in visited:
            scores[nb] = w1 * rssi[(src, nb)] + w2 * battery[nb]
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores


# =====================================================================
# NETWORK GENERATOR
# =====================================================================
DEPLOY_MODES = ["random", "grid", "circular", "cluster"]


class NetworkGenerator:
    """Vectorised position generation + KD-tree edge building."""

    @staticmethod
    def generate_positions(num_nodes: int, mode: str = "random",
                           area_size: float = 1000.0,
                           num_clusters: int = 6, cluster_std: float = 40.0,
                           seed=None) -> np.ndarray:
        rng = np.random.default_rng(seed)

        if mode == "grid":
            side    = int(math.ceil(math.sqrt(num_nodes)))
            spacing = area_size / max(side - 1, 1)
            xs, ys  = np.meshgrid(np.arange(side), np.arange(side))
            pts     = np.column_stack([xs.ravel(), ys.ravel()]) * spacing
            pos     = pts[:num_nodes].astype(float)

        elif mode == "circular":
            radius   = area_size / 2.0
            rings    = max(1, num_nodes // 20 + 1)
            per_ring = math.ceil(num_nodes / rings)
            pos      = np.zeros((num_nodes, 2))
            idx      = 0
            for r_i in range(rings):
                r     = radius * (r_i + 1) / rings
                count = min(per_ring, num_nodes - idx)
                if count <= 0:
                    break
                ang   = np.linspace(0, 2 * math.pi, count, endpoint=False)
                pos[idx:idx + count, 0] = radius + r * np.cos(ang)
                pos[idx:idx + count, 1] = radius + r * np.sin(ang)
                idx  += count

        elif mode == "cluster":
            nc      = min(num_clusters, max(1, num_nodes // 3))
            centers = rng.uniform(area_size * 0.1, area_size * 0.9, size=(nc, 2))
            assign  = rng.integers(0, nc, size=num_nodes)
            pos     = centers[assign] + rng.normal(0, cluster_std, size=(num_nodes, 2))
            pos     = np.clip(pos, 0, area_size)

        else:  # "random"
            pos = rng.uniform(0, area_size, size=(num_nodes, 2))

        return pos

    @staticmethod
    def max_range_for_sf(sf: int, Pt: float = 14, freq: float = 433e6,
                         n: float = 3.0) -> float:
        """Invert compute_rssi at SF sensitivity threshold."""
        sens = SF_SENS[sf]
        PL0  = 32.44 + 20 * math.log10(freq / 1e6)
        exp  = (Pt - sens - PL0) / (10 * n)
        return 10 ** exp

    @staticmethod
    def build_edges(positions: np.ndarray, max_range: float,
                    edge_cap: int = 60_000):
        n      = len(positions)
        capped = False

        if HAVE_SCIPY and n > 15:
            tree  = cKDTree(positions)
            pairs = list(tree.query_pairs(r=max_range))
        else:
            pairs = []
            for i in range(n):
                for j in range(i + 1, n):
                    if float(np.linalg.norm(positions[i] - positions[j])) <= max_range:
                        pairs.append((i, j))

        if len(pairs) > edge_cap:
            pairs  = random.sample(pairs, edge_cap)
            capped = True

        return pairs, capped

    @staticmethod
    def build_network(num_nodes: int, mode: str, area_size: float,
                      max_range: float, battery_init: np.ndarray,
                      seed=None):
        pos    = NetworkGenerator.generate_positions(num_nodes, mode=mode,
                                                     area_size=area_size, seed=seed)
        edges, _ = NetworkGenerator.build_edges(pos, max_range)

        neighbors = {i: set() for i in range(num_nodes)}
        rssi_map  = {}
        for (u, v) in edges:
            d = float(np.linalg.norm(pos[u] - pos[v]))
            r = compute_rssi(d)
            rssi_map[(u, v)] = rssi_map[(v, u)] = r
            neighbors[u].add(v)
            neighbors[v].add(u)

        return pos, edges, neighbors, rssi_map, battery_init.copy()


# =====================================================================
# ENERGY MODEL
# =====================================================================
class EnergyModel:
    def __init__(self, num_nodes: int, tx_cost=2.0, rx_cost=1.2,
                 idle_cost=0.05, sleep_cost=0.01):
        self.tx_cost    = tx_cost
        self.rx_cost    = rx_cost
        self.idle_cost  = idle_cost
        self.sleep_cost = sleep_cost
        self.consumed   = np.zeros(num_nodes)

    def _apply(self, battery, node, cost):
        battery[node]       = max(0.0, battery[node] - cost)
        self.consumed[node] += cost
        return cost

    def apply_tx(self, battery, node):   return self._apply(battery, node, self.tx_cost)
    def apply_rx(self, battery, node):   return self._apply(battery, node, self.rx_cost)
    def apply_idle(self, battery, node): return self._apply(battery, node, self.idle_cost)


# =====================================================================
# CSMA/CA COLLISION SIMULATOR
# =====================================================================
class CSMASimulator:
    """Simplified CSMA/CA: channel sensing, collision, exponential backoff."""

    def __init__(self, energy_model: EnergyModel, slot_time=0.01, max_retries=5):
        self.energy      = energy_model
        self.slot_time   = slot_time
        self.max_retries = max_retries
        self.reset_stats()

    def reset_stats(self):
        self.stats = dict(attempts=0, successful=0, collisions=0,
                          dropped=0, retransmissions=0, busy_channel=0)

    def simulate_hop(self, sender: int, receiver: int, battery: np.ndarray,
                     rng, contenders: int = 0) -> bool:
        retries = 0
        success = False
        while retries <= self.max_retries:
            self.stats["attempts"] += 1
            # Channel sensing
            if rng.random() < min(0.5, 0.05 * contenders):
                self.stats["busy_channel"]     += 1
                self.stats["retransmissions"]  += 1
                self.energy.apply_idle(battery, sender)
                retries += 1
                continue
            # Transmission + collision check
            self.energy.apply_tx(battery, sender)
            self.energy.apply_rx(battery, receiver)
            if rng.random() < min(0.9, 0.08 * contenders):
                self.stats["collisions"]       += 1
                self.stats["retransmissions"]  += 1
                retries += 1
            else:
                success = True
                self.stats["successful"] += 1
                break

        if not success:
            self.stats["dropped"] += 1
        return success

    def pdr(self) -> float:
        d = max(1, self.stats["successful"] + self.stats["dropped"])
        return self.stats["successful"] / d

    def collision_rate(self) -> float:
        return self.stats["collisions"] / max(1, self.stats["attempts"])

    def channel_utilization(self) -> float:
        return self.stats["busy_channel"] / max(1, self.stats["attempts"])


# =====================================================================
# SINGLE SIMULATION RUN
# =====================================================================
def run_single_simulation(num_nodes: int, area_size: float, mode: str,
                           battery_init: np.ndarray, max_range: float,
                           w1: float, w2: float, seed=None) -> dict | None:
    """
    Builds one network and routes src→dst using custom_greedy with (w1,w2).
    Returns a metrics dict, or None if the network is too sparse to route.
    """
    rng = np.random.default_rng(seed)

    pos, edges, neighbors, rssi_map, battery = NetworkGenerator.build_network(
        num_nodes, mode, area_size, max_range, battery_init, seed=seed)

    if num_nodes < 2 or not edges:
        return None

    # Pick connected src / dst
    connected = [i for i in range(num_nodes) if neighbors[i]]
    if len(connected) < 2:
        return None
    src, dst = random.sample(connected, 2)

    energy_model = EnergyModel(num_nodes)
    csma         = CSMASimulator(energy_model)

    path      = [src]
    visited   = {src}
    current   = src
    max_hops  = num_nodes * 2
    hop_count = 0
    reached   = False
    rssi_vals = []

    while current != dst and hop_count < max_hops:
        nxt, scores = custom_greedy(current, neighbors, rssi_map, battery,
                                    visited, w1=w1, w2=w2)
        if nxt is None:
            break
        contenders = min(len(neighbors[current]), 10)
        csma.simulate_hop(current, nxt, battery, rng, contenders=contenders)
        # Idle energy for overheard nodes
        for other in neighbors[current]:
            if other not in (current, nxt):
                energy_model.apply_idle(battery, other)
        if (current, nxt) in rssi_map:
            rssi_vals.append(rssi_map[(current, nxt)])
        path.append(nxt)
        visited.add(nxt)
        current    = nxt
        hop_count += 1
        if current == dst:
            reached = True
            break

    return {
        "pdr":                    csma.pdr(),
        "total_energy":           float(energy_model.consumed.sum()),
        "collision_rate":         csma.collision_rate(),
        "channel_utilization":    csma.channel_utilization(),
        "hop_count":              hop_count,
        "reached":                reached,
        "mean_battery_remaining": float(battery.mean()),
        "mean_rssi":              float(np.mean(rssi_vals)) if rssi_vals else -999.0,
        "mean_neighbors":         float(np.mean([len(v) for v in neighbors.values()])),
        "area_size":              area_size,
        "num_nodes":              num_nodes,
        "w1":                     w1,
        "w2":                     w2,
        "path":                   path,
        "positions":              pos,
        "edges":                  edges,
        "battery":                battery,
        "src":                    src,
        "dst":                    dst,
    }


# =====================================================================
# MONTE CARLO ENGINE
# =====================================================================
class MonteCarloEngine:
    """
    For each (w1, w2) candidate, runs n_trials independent simulations
    across randomised environments and returns averaged performance metrics.

    All simulation parameters (area, nodes, battery, deployment) are drawn
    uniformly at random within the configured ranges.
    """

    def __init__(self, n_trials: int,
                 area_range:    tuple = (10.0, 1000.0),
                 node_range:    tuple = (10,   100),
                 battery_range: tuple = (10.0, 100.0),
                 sf:            int   = 9):
        self.n_trials      = n_trials
        self.area_range    = area_range
        self.node_range    = node_range
        self.battery_range = battery_range
        self.sf            = sf
        self.max_range     = NetworkGenerator.max_range_for_sf(sf)

    def run_for_weights(self, w1: float, w2: float,
                        progress_cb=None) -> dict | None:
        """Runs n_trials for (w1, w2). Returns averaged metric dict."""
        results = []
        for t in range(self.n_trials):
            area    = random.uniform(*self.area_range)
            n_nodes = random.randint(*self.node_range)
            batt_lo = random.uniform(*self.battery_range)
            batt_hi = random.uniform(batt_lo, self.battery_range[1])
            batt    = np.random.uniform(batt_lo, batt_hi, size=n_nodes)
            mode    = random.choice(DEPLOY_MODES)
            # Scale effective range to guarantee reasonable connectivity
            eff_range = float(np.clip(self.max_range, area * 0.12, area * 0.38))

            r = run_single_simulation(n_nodes, area, mode, batt, eff_range, w1, w2)
            if r is not None:
                results.append(r)
            if progress_cb:
                progress_cb(t + 1, self.n_trials)

        if not results:
            return None

        return {
            "w1":                     w1,
            "w2":                     w2,
            "mean_pdr":               float(np.mean([r["pdr"]                    for r in results])),
            "mean_energy":            float(np.mean([r["total_energy"]           for r in results])),
            "mean_collision_rate":    float(np.mean([r["collision_rate"]         for r in results])),
            "mean_channel_util":      float(np.mean([r["channel_utilization"]    for r in results])),
            "mean_hop_count":         float(np.mean([r["hop_count"]              for r in results])),
            "reach_rate":             float(np.mean([float(r["reached"])         for r in results])),
            "mean_battery_remaining": float(np.mean([r["mean_battery_remaining"] for r in results])),
            "mean_rssi":              float(np.mean([r["mean_rssi"]              for r in results])),
            "mean_neighbors":         float(np.mean([r["mean_neighbors"]         for r in results])),
            "n_valid":                len(results),
        }


# =====================================================================
# WEIGHT CANDIDATES  (two-phase search)
# =====================================================================
def _coarse_candidates(n: int) -> list[tuple[float, float]]:
    return [(round(w1, 3), round(1.0 - w1, 3))
            for w1 in np.linspace(0.05, 0.95, n)]


def _fine_candidates(coarse_results: list, top_k: int,
                     n_fine: int) -> list[tuple[float, float]]:
    top = sorted(coarse_results, key=lambda r: r["mean_pdr"], reverse=True)[:top_k]
    fine = []
    per  = max(1, n_fine // top_k)
    for r in top:
        for _ in range(per):
            w1 = float(np.clip(r["w1"] + random.uniform(-0.10, 0.10), 0.05, 0.95))
            fine.append((round(w1, 4), round(1.0 - w1, 4)))
    return fine


# =====================================================================
# EMERGENCY PROFILER  (MLP + KMeans on MC results)
# =====================================================================
class EmergencyProfiler:
    """
    Trains an MLPRegressor on the full MC result dataset, then uses K-Means
    to discover three scenario clusters from the results.  The MLP is queried
    on each cluster centroid to produce the optimal (w1, w2) for that scenario.

    Feature vector:
        [mean_battery_remaining, mean_rssi, mean_neighbors,
         mean_collision_rate, mean_channel_util, mean_pdr]
    Label vector:
        [w1, w2]

    Emergency profile assignment (from centroid characteristics):
        E1 – cluster whose centroid has the lowest mean_battery_remaining
        E2 – cluster whose centroid has the highest mean_collision_rate
        E3 – cluster whose centroid has the lowest mean_rssi
    """

    PROFILE_META = [
        ("E1 – Low Battery Emergency",   "🔋", "#ff6b35"),
        ("E2 – High Collision Emergency", "💥", "#ff4d4d"),
        ("E3 – Range-Limited Emergency",  "📶", "#4d79ff"),
    ]

    def __init__(self):
        self.model    = None
        self.scaler   = None
        self.trained  = False
        self.profiles = []

    # ── dataset construction ──────────────────────────────────────────
    def _build_dataset(self, mc_results):
        X, y = [], []
        for r in mc_results:
            X.append([
                r["mean_battery_remaining"],
                r["mean_rssi"],
                r["mean_neighbors"],
                r["mean_collision_rate"],
                r["mean_channel_util"],
                r["mean_pdr"],
            ])
            y.append([r["w1"], r["w2"]])
        return np.array(X, dtype=float), np.array(y, dtype=float)

    # ── training ─────────────────────────────────────────────────────
    def train(self, mc_results: list, n_clusters: int = 3):
        if not HAVE_SKLEARN:
            return False, "scikit-learn not installed"
        if len(mc_results) < n_clusters + 2:
            return False, "Too few MC data points – run more optimizer trials first"

        X, y = self._build_dataset(mc_results)

        self.scaler = StandardScaler()
        X_s         = self.scaler.fit_transform(X)

        # ── MLP regressor ─────────────────────────────────────────────
        self.model = MLPRegressor(
            hidden_layer_sizes=(64, 64),
            activation="relu",
            solver="adam",
            learning_rate_init=1e-3,
            max_iter=1200,
            random_state=42,
            early_stopping=True,
            validation_fraction=0.15,
            n_iter_no_change=30,
        )
        self.model.fit(X_s, y)

        # ── K-Means clustering on result features ─────────────────────
        km          = KMeans(n_clusters=n_clusters, random_state=42, n_init=15)
        labels      = km.fit_predict(X_s)
        centroids_s = km.cluster_centers_            # shape (k, n_features) scaled
        centroids_r = self.scaler.inverse_transform(centroids_s)  # raw values

        # Predict optimal weights for each centroid
        pred_w = self.model.predict(centroids_s)          # (k, 2)
        pred_w = np.clip(pred_w, 0.05, 0.95)
        # Normalise so w1 + w2 = 1.0
        pred_w = pred_w / pred_w.sum(axis=1, keepdims=True)

        # ── Assign scenarios to clusters ──────────────────────────────
        batt_col      = centroids_r[:, 0]   # mean_battery_remaining
        rssi_col      = centroids_r[:, 1]   # mean_rssi
        collision_col = centroids_r[:, 3]   # mean_collision_rate

        assigned      = set()
        scenario_map  = {}

        # E1 → lowest battery
        e1 = int(np.argmin(batt_col))
        scenario_map[0] = e1;  assigned.add(e1)

        # E2 → highest collision (from remaining)
        remaining = [i for i in range(n_clusters) if i not in assigned]
        e2        = remaining[int(np.argmax(collision_col[remaining]))]
        scenario_map[1] = e2;  assigned.add(e2)

        # E3 → lowest RSSI (from remaining)
        remaining = [i for i in range(n_clusters) if i not in assigned]
        if remaining:
            e3 = remaining[int(np.argmin(rssi_col[remaining]))]
        else:
            e3 = e1  # fallback (degenerate)
        scenario_map[2] = e3

        # ── Build profile dicts ───────────────────────────────────────
        self.profiles = []
        for profile_idx in range(3):
            cidx    = scenario_map[profile_idx]
            w1, w2  = float(pred_w[cidx, 0]), float(pred_w[cidx, 1])
            centroid = centroids_r[cidx]
            name, icon, color = self.PROFILE_META[profile_idx]
            cluster_size = int(np.sum(labels == cidx))

            self.profiles.append({
                "name":               name,
                "icon":               icon,
                "color":              color,
                "w1":                 round(w1, 4),
                "w2":                 round(w2, 4),
                "cluster_idx":        cidx,
                "cluster_size":       cluster_size,
                "centroid_battery":   round(float(centroid[0]), 2),
                "centroid_rssi":      round(float(centroid[1]), 2),
                "centroid_neighbors": round(float(centroid[2]), 2),
                "centroid_collision": round(float(centroid[3]), 4),
                "centroid_util":      round(float(centroid[4]), 4),
                "centroid_pdr":       round(float(centroid[5]), 4),
            })

        self.trained = True
        return True, f"MLP trained on {len(mc_results)} samples; {n_clusters} clusters found"


# =====================================================================
# VISUALIZATION MANAGER
# =====================================================================
class VisualizationManager:
    BG = "#0a0a18"

    def __init__(self, ax):
        self.ax = ax

    def draw(self, positions, edges, battery, path=None,
             src=None, dst=None, title=""):
        ax = self.ax
        ax.clear()
        ax.set_facecolor(self.BG)
        n  = len(positions)
        xs, ys = positions[:, 0], positions[:, 1]

        # ── Edges ──
        edge_alpha = 0.18 if n > 150 else 0.40
        edge_lw    = 0.25 if n > 150 else 0.6
        for (u, v) in edges:
            ax.plot([xs[u], xs[v]], [ys[u], ys[v]],
                    color="#3a3a6a", linewidth=edge_lw, alpha=edge_alpha, zorder=1)

        # ── Nodes ──
        node_s  = 10 if n > 150 else (80 if n > 50 else 180)
        scatter = ax.scatter(xs, ys, s=node_s, c=battery, cmap="RdYlGn",
                              vmin=0, vmax=100, edgecolors="none",
                              alpha=0.85, zorder=2)
        if n <= 40:
            for i in range(n):
                ax.annotate(str(i), (xs[i], ys[i]), ha="center", va="center",
                             fontsize=6, color="white", zorder=3)

        # ── Path ──
        if path and len(path) > 1:
            px = [positions[nd][0] for nd in path]
            py = [positions[nd][1] for nd in path]
            ax.plot(px, py, color="#ff4d4d", linewidth=2.8, zorder=4, alpha=0.92)
            ax.scatter(px, py, s=55, c="#ffaa00", edgecolors="#ff4d4d",
                        linewidths=0.8, zorder=5)

        # ── Src / Dst markers ──
        if src is not None:
            ax.scatter([xs[src]], [ys[src]], s=300, c="#00ff88",
                        edgecolors="white", linewidths=1.5, zorder=6, marker="*")
        if dst is not None:
            ax.scatter([xs[dst]], [ys[dst]], s=200, c="#ff00cc",
                        edgecolors="white", linewidths=1.5, zorder=6, marker="D")

        # ── Styling ──
        ax.set_title(title or f"Network  ({n} nodes, {len(edges)} links)",
                      color="white", fontsize=10, pad=8)
        ax.tick_params(colors="#666")
        for sp in ax.spines.values():
            sp.set_edgecolor("#222")

        return scatter


# =====================================================================
# MAIN APPLICATION GUI
# =====================================================================
class OptimizerApp:
    """4-tab Tkinter application: Optimizer / Emergency Profiles / Results / Network."""

    # Dark palette
    BG       = "#0a0a18"
    BG2      = "#12122a"
    BG3      = "#1a1a3e"
    ACCENT   = "#00d4ff"
    ACCENT2  = "#7c3aed"
    FG       = "#d0d0f0"
    FG_DIM   = "#7070a0"
    SUCCESS  = "#00ff88"
    WARN     = "#ffaa00"
    DANGER   = "#ff4d4d"

    def __init__(self, root: tk.Tk):
        self.root      = root
        self.root.title("LoRa AI Weight Optimizer")
        self.root.geometry("1340x860")
        self.root.configure(bg=self.BG)

        # State
        self.mc_results       : list  = []
        self.best_result      : dict | None  = None
        self.profiler         = EmergencyProfiler()
        self.last_sim_result  : dict | None  = None
        self._worker_thread   : threading.Thread | None = None
        self._stop_flag       : bool  = False

        self._setup_style()
        self._build_header()
        self._build_tabs()

        if not HAVE_SKLEARN:
            messagebox.showwarning(
                "Missing Dependency",
                "scikit-learn not found.\n"
                "Run:  pip install scikit-learn\n\n"
                "The Emergency Profiles tab will be disabled.")
        if not HAVE_SCIPY:
            messagebox.showwarning(
                "Performance Notice",
                "scipy not found – edge building will be slow for large networks.\n"
                "Run:  pip install scipy")

    # ──────────────────────────────────────────────────────────────────
    # STYLE
    # ──────────────────────────────────────────────────────────────────
    def _setup_style(self):
        s = ttk.Style()
        s.theme_use("clam")
        s.configure("TNotebook",            background=self.BG,  borderwidth=0)
        s.configure("TNotebook.Tab",        background=self.BG2, foreground=self.FG_DIM,
                     padding=[18, 9],       font=("Segoe UI", 10, "bold"))
        s.map("TNotebook.Tab",
              background=[("selected", self.BG3)],
              foreground=[("selected", self.ACCENT)])
        s.configure("TFrame",              background=self.BG)
        s.configure("TLabel",              background=self.BG, foreground=self.FG,
                     font=("Segoe UI", 9))
        s.configure("TLabelframe",         background=self.BG, foreground=self.ACCENT,
                     bordercolor=self.BG2)
        s.configure("TLabelframe.Label",   background=self.BG, foreground=self.ACCENT,
                     font=("Segoe UI", 9, "bold"))
        s.configure("Horizontal.TProgressbar",
                     background=self.ACCENT, troughcolor=self.BG2,
                     bordercolor=self.BG)

    # ──────────────────────────────────────────────────────────────────
    # HEADER
    # ──────────────────────────────────────────────────────────────────
    def _build_header(self):
        hdr = tk.Frame(self.root, bg=self.BG2, height=58)
        hdr.pack(fill="x", side="top")
        hdr.pack_propagate(False)

        tk.Label(hdr, text="⚡  LoRa AI Weight Optimizer",
                  font=("Segoe UI", 17, "bold"),
                  bg=self.BG2, fg=self.ACCENT).pack(side="left", padx=22, pady=10)

        tk.Label(hdr,
                  text="Monte Carlo · MLP · Emergency Profiles · Multi-Hop LoRa",
                  font=("Segoe UI", 9), bg=self.BG2, fg=self.FG_DIM
                  ).pack(side="left", padx=0)

        self.status_var = tk.StringVar(value="Ready")
        tk.Label(hdr, textvariable=self.status_var,
                  font=("Segoe UI", 9, "italic"),
                  bg=self.BG2, fg=self.FG_DIM).pack(side="right", padx=22)

    # ──────────────────────────────────────────────────────────────────
    # TABS
    # ──────────────────────────────────────────────────────────────────
    def _build_tabs(self):
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(fill="both", expand=True, padx=10, pady=8)

        self.tab_opt   = ttk.Frame(self.nb)
        self.tab_emerg = ttk.Frame(self.nb)
        self.tab_res   = ttk.Frame(self.nb)
        self.tab_net   = ttk.Frame(self.nb)

        self.nb.add(self.tab_opt,   text="🔧  Optimizer")
        self.nb.add(self.tab_emerg, text="🚨  Emergency Profiles")
        self.nb.add(self.tab_res,   text="📊  Results")
        self.nb.add(self.tab_net,   text="🌐  Network View")

        self._build_optimizer_tab()
        self._build_emergency_tab()
        self._build_results_tab()
        self._build_network_tab()

    # ══════════════════════════════════════════════════════════════════
    # TAB 1 – OPTIMIZER
    # ══════════════════════════════════════════════════════════════════
    def _build_optimizer_tab(self):
        tab  = self.tab_opt
        left = tk.Frame(tab, bg=self.BG, width=400)
        left.pack(side="left", fill="y", padx=(12, 6), pady=10)
        left.pack_propagate(False)

        right = tk.Frame(tab, bg=self.BG)
        right.pack(side="right", fill="both", expand=True, padx=(6, 12), pady=10)

        # ── MC Configuration ──────────────────────────────────────────
        mc = ttk.LabelFrame(left, text="Monte Carlo Configuration")
        mc.pack(fill="x", pady=(0, 8))

        rows = [
            ("MC Trials per Candidate (manual):", "mc_trials",   50,   5, 2000),
            ("Coarse Grid Points (w1 sweep):",    "n_coarse",    10,   5,   30),
            ("Fine Random Candidates:",           "n_fine",      30,   5,  150),
            ("Top-K Seeds for Fine Phase:",       "top_k",        5,   1,   15),
        ]
        self.mc_trials_var = tk.IntVar(value=50)
        self.n_coarse_var  = tk.IntVar(value=10)
        self.n_fine_var    = tk.IntVar(value=30)
        self.top_k_var     = tk.IntVar(value=5)
        vars_  = [self.mc_trials_var, self.n_coarse_var, self.n_fine_var, self.top_k_var]

        for i, (label, _, default, lo, hi) in enumerate(rows):
            tk.Label(mc, text=label, bg=self.BG, fg=self.FG,
                      font=("Segoe UI", 9)).grid(row=i, column=0, sticky="w", padx=8, pady=3)
            self._spin(mc, vars_[i], lo, hi).grid(row=i, column=1, sticky="w", padx=6, pady=3)

        # ── Environment ───────────────────────────────────────────────
        env = ttk.LabelFrame(left, text="Simulation Environment  (randomised per trial)")
        env.pack(fill="x", pady=(0, 8))

        self.area_min_var  = tk.DoubleVar(value=10.0)
        self.area_max_var  = tk.DoubleVar(value=1000.0)
        self.nodes_min_var = tk.IntVar(value=10)
        self.nodes_max_var = tk.IntVar(value=80)
        self.batt_min_var  = tk.DoubleVar(value=10.0)
        self.batt_max_var  = tk.DoubleVar(value=100.0)
        self.sf_var        = tk.IntVar(value=9)

        env_rows = [
            ("Area Min (m):",       self.area_min_var,  "entry"),
            ("Area Max (m):",       self.area_max_var,  "entry"),
            ("Nodes Min:",          self.nodes_min_var, "spin5_500"),
            ("Nodes Max:",          self.nodes_max_var, "spin5_500"),
            ("Battery Min (%):",    self.batt_min_var,  "entry"),
            ("Battery Max (%):",    self.batt_max_var,  "entry"),
            ("Spreading Factor:",   self.sf_var,        "combo"),
        ]
        for i, (label, var, kind) in enumerate(env_rows):
            tk.Label(env, text=label, bg=self.BG, fg=self.FG,
                      font=("Segoe UI", 9)).grid(row=i, column=0, sticky="w", padx=8, pady=3)
            if kind == "entry":
                self._entry(env, var).grid(row=i, column=1, sticky="w", padx=6, pady=3)
            elif kind == "spin5_500":
                self._spin(env, var, 5, 500).grid(row=i, column=1, sticky="w", padx=6, pady=3)
            elif kind == "combo":
                c = ttk.Combobox(env, textvariable=var, values=list(SF_SENS.keys()),
                                   width=7, state="readonly")
                c.grid(row=i, column=1, sticky="w", padx=6, pady=3)

        # ── Progress ──────────────────────────────────────────────────
        prog = ttk.LabelFrame(left, text="Progress")
        prog.pack(fill="x", pady=(0, 8))
        self.prog_var   = tk.DoubleVar(value=0)
        self.prog_bar   = ttk.Progressbar(prog, variable=self.prog_var, maximum=100,
                                            style="Horizontal.TProgressbar", length=360)
        self.prog_bar.pack(padx=10, pady=(6, 2), fill="x")
        self.prog_label = tk.Label(prog, text="Idle", bg=self.BG, fg=self.FG_DIM,
                                    font=("Segoe UI", 9))
        self.prog_label.pack(pady=(2, 8))

        # ── Buttons ───────────────────────────────────────────────────
        btn_frame = tk.Frame(left, bg=self.BG)
        btn_frame.pack(fill="x", pady=4)
        self._btn(btn_frame, "▶  Run Optimizer",     self._run_optimizer,  self.ACCENT,  self.BG).pack(fill="x", pady=3)
        self._btn(btn_frame, "⏹  Stop",              self._stop_optimizer, self.DANGER, "white").pack(fill="x", pady=3)
        self._btn(btn_frame, "🔄  Clear Results",    self._clear_results,  self.BG3,    self.FG).pack(fill="x", pady=3)
        self._btn(btn_frame, "💾  Export CSV",        self._export_csv,     self.BG3,    self.FG).pack(fill="x", pady=3)

        # ── Best weights banner ───────────────────────────────────────
        best_f = ttk.LabelFrame(left, text="Best Weights Found")
        best_f.pack(fill="x", pady=(8, 0))
        self.best_label = tk.Label(best_f, text="Not yet optimized",
                                    bg=self.BG, fg=self.SUCCESS,
                                    font=("Segoe UI", 11, "bold"),
                                    wraplength=360, justify="left")
        self.best_label.pack(padx=10, pady=10)

        # ── Right: live PDR chart ─────────────────────────────────────
        self.opt_fig, self.opt_ax = plt.subplots(figsize=(5.8, 5.8), facecolor=self.BG)
        self.opt_ax.set_facecolor(self.BG)
        self.opt_canvas = FigureCanvasTkAgg(self.opt_fig, right)
        self.opt_canvas.get_tk_widget().pack(fill="both", expand=True)
        self._draw_opt_chart_empty()

    def _draw_opt_chart_empty(self):
        ax = self.opt_ax
        ax.clear()
        ax.set_facecolor(self.BG)
        ax.text(0.5, 0.5, "Run the optimizer to see\nPDR vs w1 chart",
                 ha="center", va="center", color=self.FG_DIM,
                 fontsize=12, transform=ax.transAxes)
        ax.set_title("PDR vs w1", color=self.FG, fontsize=11)
        for sp in ax.spines.values(): sp.set_edgecolor("#333")
        ax.tick_params(colors="#555")
        self.opt_canvas.draw()

    def _update_opt_chart(self):
        ax = self.opt_ax
        ax.clear()
        ax.set_facecolor(self.BG)
        if not self.mc_results:
            return

        sorted_r = sorted(self.mc_results, key=lambda r: r["w1"])
        w1s  = [r["w1"]            for r in sorted_r]
        pdrs = [r["mean_pdr"]      for r in sorted_r]
        cols = [r["mean_collision_rate"] for r in sorted_r]

        ax.plot(w1s, pdrs, color=self.ACCENT, linewidth=2.2,
                 marker="o", markersize=5, label="Mean PDR")
        ax2 = ax.twinx()
        ax2.set_facecolor(self.BG)
        ax2.plot(w1s, cols, color=self.DANGER, linewidth=1.5,
                  marker="s", markersize=4, linestyle="--", label="Collision Rate")
        ax2.set_ylabel("Collision Rate", color=self.DANGER, fontsize=8)
        ax2.tick_params(colors=self.DANGER)
        ax2.spines["right"].set_edgecolor("#444")

        if self.best_result:
            ax.axvline(x=self.best_result["w1"], color=self.SUCCESS,
                        linestyle="--", linewidth=1.6,
                        label=f"Best w1={self.best_result['w1']:.3f}")

        ax.set_title("PDR & Collision Rate vs w1", color="white", fontsize=10)
        ax.set_xlabel("w1 (RSSI weight)", color=self.FG)
        ax.set_ylabel("Mean PDR",          color=self.FG)
        ax.tick_params(colors=self.FG_DIM)
        for sp in ax.spines.values(): sp.set_edgecolor("#333")

        lines1, lbls1 = ax.get_legend_handles_labels()
        lines2, lbls2 = ax2.get_legend_handles_labels()
        ax.legend(lines1 + lines2, lbls1 + lbls2,
                   facecolor=self.BG2, edgecolor="#444",
                   labelcolor="white", fontsize=8)
        self.opt_canvas.draw()

    # ══════════════════════════════════════════════════════════════════
    # TAB 2 – EMERGENCY PROFILES
    # ══════════════════════════════════════════════════════════════════
    def _build_emergency_tab(self):
        tab = self.tab_emerg

        hdr = tk.Frame(tab, bg=self.BG)
        hdr.pack(fill="x", padx=14, pady=(12, 4))
        tk.Label(hdr, text="Emergency Weight Profiles",
                  font=("Segoe UI", 14, "bold"),
                  bg=self.BG, fg=self.DANGER).pack(side="left")
        self._btn(hdr, "🧠  Generate Profiles from Results",
                   self._generate_profiles, self.DANGER, "white").pack(side="right")

        self.emerg_info = tk.Label(
            tab,
            text="Run the Optimizer first, then click 'Generate Profiles'.\n"
                  "The MLP will be trained on your MC dataset and K-Means will "
                  "cluster the results to discover three emergency scenarios.",
            bg=self.BG, fg=self.FG_DIM, font=("Segoe UI", 9),
            wraplength=980, justify="left")
        self.emerg_info.pack(padx=14, pady=(0, 8))

        # MLP info banner
        mlp_f = ttk.LabelFrame(tab, text="MLP Model Info")
        mlp_f.pack(fill="x", padx=14, pady=(0, 10))
        self.mlp_info_label = tk.Label(
            mlp_f,
            text="Model not yet trained.",
            bg=self.BG, fg=self.FG_DIM, font=("Segoe UI", 9),
            wraplength=980, justify="left")
        self.mlp_info_label.pack(padx=10, pady=6)

        # Cards area
        self.cards_frame = tk.Frame(tab, bg=self.BG)
        self.cards_frame.pack(fill="both", expand=True, padx=14, pady=6)

        self.cards_placeholder = tk.Label(
            self.cards_frame,
            text="No profiles yet.",
            bg=self.BG, fg="#333355", font=("Segoe UI", 15))
        self.cards_placeholder.pack(expand=True)

    def _render_profile_cards(self):
        for w in self.cards_frame.winfo_children():
            w.destroy()

        for profile in self.profiler.profiles:
            color = profile["color"]
            card  = tk.Frame(self.cards_frame, bg=self.BG2, bd=0,
                              highlightbackground=color, highlightthickness=2)
            card.pack(side="left", fill="both", expand=True, padx=8, pady=6)

            # Header strip
            hdr_strip = tk.Frame(card, bg=color, height=36)
            hdr_strip.pack(fill="x")
            hdr_strip.pack_propagate(False)
            tk.Label(hdr_strip, text=f"{profile['icon']}  {profile['name']}",
                      font=("Segoe UI", 11, "bold"),
                      bg=color, fg="white").pack(pady=7, padx=10)

            # Info rows
            info_rows = [
                ("Optimal w1  (RSSI weight)",    f"{profile['w1']:.4f}",  self.ACCENT),
                ("Optimal w2  (Battery weight)",  f"{profile['w2']:.4f}",  self.ACCENT),
                ("Cluster size",                  f"{profile['cluster_size']} scenarios", self.FG),
                ("Centroid – Battery remaining",  f"{profile['centroid_battery']:.1f} %",  self.SUCCESS),
                ("Centroid – Mean RSSI",          f"{profile['centroid_rssi']:.2f} dBm",   self.FG),
                ("Centroid – Collision rate",     f"{profile['centroid_collision']:.4f}",  self.DANGER),
                ("Centroid – Channel util",       f"{profile['centroid_util']:.4f}",       self.WARN),
                ("Centroid – PDR",                f"{profile['centroid_pdr']:.4f}",        self.SUCCESS),
            ]
            for label, value, vc in info_rows:
                row = tk.Frame(card, bg=self.BG2)
                row.pack(fill="x", padx=14, pady=2)
                tk.Label(row, text=label, bg=self.BG2, fg=self.FG_DIM,
                          font=("Segoe UI", 8), width=30, anchor="w").pack(side="left")
                tk.Label(row, text=value, bg=self.BG2, fg=vc,
                          font=("Segoe UI", 9, "bold")).pack(side="left", padx=4)

            # Run button
            btn = self._btn(card,
                             f"▶  Simulate  (w1={profile['w1']:.4f})",
                             lambda p=profile: self._run_emergency_sim(p),
                             self.BG3, color)
            btn.pack(fill="x", padx=10, pady=(14, 12))

    # ══════════════════════════════════════════════════════════════════
    # TAB 3 – RESULTS
    # ══════════════════════════════════════════════════════════════════
    def _build_results_tab(self):
        tab = self.tab_res

        hdr = tk.Frame(tab, bg=self.BG)
        hdr.pack(fill="x", padx=14, pady=(12, 4))
        tk.Label(hdr, text="Simulation Results & Comparison",
                  font=("Segoe UI", 14, "bold"),
                  bg=self.BG, fg=self.ACCENT).pack(side="left")
        self._btn(hdr, "🔄  Refresh", self._refresh_results, self.BG3, self.ACCENT).pack(side="right")

        self.res_fig, self.res_axes = plt.subplots(2, 2, figsize=(10, 6.8),
                                                    facecolor=self.BG)
        self.res_fig.subplots_adjust(hspace=0.42, wspace=0.38)
        for ax in self.res_axes.flat:
            ax.set_facecolor(self.BG)
        self.res_canvas = FigureCanvasTkAgg(self.res_fig, tab)
        self.res_canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=6)

    def _refresh_results(self):
        if not self.mc_results:
            messagebox.showinfo("No Data", "Run the optimizer first.")
            return
        self._draw_results_charts()

    def _draw_results_charts(self):
        axes = self.res_axes
        for ax in axes.flat:
            ax.clear()
            ax.set_facecolor(self.BG)

        sorted_r   = sorted(self.mc_results, key=lambda r: r["w1"])
        w1s        = np.array([r["w1"]                    for r in sorted_r])
        pdrs       = np.array([r["mean_pdr"]               for r in sorted_r])
        energies   = np.array([r["mean_energy"]            for r in sorted_r])
        collisions = np.array([r["mean_collision_rate"]    for r in sorted_r])
        reach      = np.array([r["reach_rate"]             for r in sorted_r])

        kw = dict(color=self.FG_DIM, fontsize=8)

        # 1 – PDR
        axes[0, 0].plot(w1s, pdrs, color=self.ACCENT, lw=2, marker="o", ms=4)
        axes[0, 0].fill_between(w1s, pdrs, alpha=0.12, color=self.ACCENT)
        axes[0, 0].set_title("Mean PDR vs w1",        color="white", fontsize=9)
        axes[0, 0].set_xlabel("w1",                   **kw)
        axes[0, 0].set_ylabel("PDR",                  **kw)

        # 2 – Energy
        axes[0, 1].plot(w1s, energies, color=self.WARN, lw=2, marker="s", ms=4)
        axes[0, 1].fill_between(w1s, energies, alpha=0.12, color=self.WARN)
        axes[0, 1].set_title("Mean Energy vs w1",     color="white", fontsize=9)
        axes[0, 1].set_xlabel("w1",                   **kw)
        axes[0, 1].set_ylabel("Energy (units)",       **kw)

        # 3 – Collision rate + reach rate
        axes[1, 0].plot(w1s, collisions, color=self.DANGER, lw=2, marker="^", ms=4,
                         label="Collision Rate")
        axes[1, 0].plot(w1s, reach,      color=self.SUCCESS, lw=2, marker="v", ms=4,
                         linestyle="--",  label="Reach Rate")
        axes[1, 0].set_title("Collision & Reach Rate vs w1", color="white", fontsize=9)
        axes[1, 0].set_xlabel("w1",                          **kw)
        axes[1, 0].legend(facecolor=self.BG2, edgecolor="#444",
                           labelcolor="white", fontsize=7)

        # 4 – Profile comparison bar chart
        if self.profiler.trained and self.profiler.profiles and self.best_result:
            lbls   = ["Best\n(Normal)"] + [f"E{i+1}" for i in range(3)]
            bar_pdrs = [self.best_result["mean_pdr"]] + \
                       [p["centroid_pdr"] for p in self.profiler.profiles]
            bar_cols = [self.ACCENT, "#ff6b35", "#ff4d4d", "#4d79ff"]
            bars = axes[1, 1].bar(lbls, bar_pdrs, color=bar_cols,
                                   edgecolor="#111", width=0.55)
            for bar, val in zip(bars, bar_pdrs):
                axes[1, 1].text(bar.get_x() + bar.get_width() / 2,
                                 bar.get_height() + 0.004,
                                 f"{val:.3f}", ha="center", va="bottom",
                                 color="white", fontsize=8)
            axes[1, 1].set_title("PDR Comparison: Normal vs Profiles",
                                  color="white", fontsize=9)
            axes[1, 1].set_ylabel("PDR", **kw)
        else:
            axes[1, 1].text(0.5, 0.5,
                             "Generate emergency profiles\nto see comparison",
                             ha="center", va="center",
                             color=self.FG_DIM, fontsize=10,
                             transform=axes[1, 1].transAxes)
            axes[1, 1].set_title("Profile Comparison", color="white", fontsize=9)

        for ax in axes.flat:
            ax.tick_params(colors=self.FG_DIM)
            for sp in ax.spines.values():
                sp.set_edgecolor("#333")

        self.res_canvas.draw()

    # ══════════════════════════════════════════════════════════════════
    # TAB 4 – NETWORK VIEW
    # ══════════════════════════════════════════════════════════════════
    def _build_network_tab(self):
        tab = self.tab_net

        hdr = tk.Frame(tab, bg=self.BG)
        hdr.pack(fill="x", padx=14, pady=(12, 4))
        tk.Label(hdr, text="Network Visualization",
                  font=("Segoe UI", 14, "bold"),
                  bg=self.BG, fg=self.ACCENT).pack(side="left")
        self._btn(hdr, "🎯  Run Best Weights Sim",
                   self._run_best_sim, self.ACCENT, self.BG).pack(side="right", padx=4)

        self.net_info = tk.Label(tab, text="Run a simulation to view the network.",
                                  bg=self.BG, fg=self.FG_DIM, font=("Segoe UI", 9))
        self.net_info.pack(padx=14, pady=(0, 4))

        # Colorbar legend row
        legend_row = tk.Frame(tab, bg=self.BG)
        legend_row.pack(fill="x", padx=14, pady=(0, 4))
        tk.Label(legend_row, text="Node colour = Battery level   ",
                  bg=self.BG, fg=self.FG_DIM, font=("Segoe UI", 8)).pack(side="left")
        tk.Label(legend_row, text="★ = Source   ",  bg=self.BG,
                  fg=self.SUCCESS, font=("Segoe UI", 8, "bold")).pack(side="left")
        tk.Label(legend_row, text="◆ = Destination   ", bg=self.BG,
                  fg="#ff00cc", font=("Segoe UI", 8, "bold")).pack(side="left")
        tk.Label(legend_row, text="━ Red path = route taken",
                  bg=self.BG, fg=self.DANGER, font=("Segoe UI", 8, "bold")).pack(side="left")

        self.net_fig, self.net_ax = plt.subplots(figsize=(9.5, 6.5), facecolor=self.BG)
        self.net_ax.set_facecolor(self.BG)
        self.net_canvas = FigureCanvasTkAgg(self.net_fig, tab)
        self.net_canvas.get_tk_widget().pack(fill="both", expand=True, padx=10, pady=4)
        NavigationToolbar2Tk(self.net_canvas, tab).update()

        self.net_viz = VisualizationManager(self.net_ax)

        # Colourbar placeholder (will be replaced on each draw)
        self._net_cbar = None

    def _draw_network_tab(self, r: dict, title: str = ""):
        self.net_ax.clear()
        if self._net_cbar:
            try:
                self._net_cbar.remove()
            except Exception:
                pass
            self._net_cbar = None

        sc = self.net_viz.draw(
            r["positions"], r["edges"], r["battery"],
            path=r.get("path"), src=r.get("src"), dst=r.get("dst"),
            title=title)

        self._net_cbar = self.net_fig.colorbar(sc, ax=self.net_ax,
                                                 label="Battery %",
                                                 fraction=0.028, pad=0.02)
        self._net_cbar.ax.yaxis.set_tick_params(color=self.FG_DIM)
        self._net_cbar.outline.set_edgecolor("#333")
        plt.setp(self._net_cbar.ax.yaxis.get_ticklabels(), color=self.FG_DIM)

        self.net_fig.tight_layout()
        self.net_canvas.draw()

        reached_str = "✅ Reached destination" if r.get("reached") else "❌ Did not reach"
        info = (f"Nodes: {r['num_nodes']}  ·  Edges: {len(r['edges'])}  ·  "
                f"Hops: {r['hop_count']}  ·  {reached_str}  ·  "
                f"PDR: {r['pdr']:.3f}  ·  Energy: {r['total_energy']:.1f}  ·  "
                f"w1={r['w1']:.4f}  w2={r['w2']:.4f}")
        self.net_info.config(text=info, fg=self.ACCENT)

    # ──────────────────────────────────────────────────────────────────
    # ACTIONS – OPTIMIZER
    # ──────────────────────────────────────────────────────────────────
    def _run_optimizer(self):
        if self._worker_thread and self._worker_thread.is_alive():
            messagebox.showinfo("Already Running", "Optimizer is running. Stop it first.")
            return
        self._stop_flag = False
        self._worker_thread = threading.Thread(target=self._optimizer_worker, daemon=True)
        self._worker_thread.start()

    def _optimizer_worker(self):
        try:
            n_trials  = self.mc_trials_var.get()
            n_coarse  = self.n_coarse_var.get()
            n_fine    = self.n_fine_var.get()
            top_k     = self.top_k_var.get()
            sf        = self.sf_var.get()
            engine    = MonteCarloEngine(
                n_trials      = n_trials,
                area_range    = (self.area_min_var.get(),  self.area_max_var.get()),
                node_range    = (self.nodes_min_var.get(), self.nodes_max_var.get()),
                battery_range = (self.batt_min_var.get(),  self.batt_max_var.get()),
                sf            = sf,
            )

            coarse_cands = _coarse_candidates(n_coarse)
            fine_cands   = []
            total_sims   = (len(coarse_cands) + n_fine) * n_trials

            self._set_status("Phase 1 – Coarse Grid Search …")
            coarse_results = []
            done = [0]

            for ci, (w1, w2) in enumerate(coarse_cands):
                if self._stop_flag:
                    break
                def _cb(t, _tot, ci=ci, done=done):
                    pct = min(99, (done[0] + t) / max(1, total_sims) * 100)
                    self._after(lambda p=pct, c=ci, t_=t:
                        (self.prog_var.set(p),
                         self.prog_label.config(
                             text=f"Phase 1 · Cand {c+1}/{len(coarse_cands)} · Trial {t_}/{n_trials}")))
                r = engine.run_for_weights(w1, w2, progress_cb=_cb)
                if r:
                    coarse_results.append(r)
                done[0] += n_trials

            self.mc_results.extend(coarse_results)

            if not self._stop_flag and coarse_results:
                self._set_status("Phase 2 – Fine Random Search …")
                fine_cands = _fine_candidates(coarse_results, top_k, n_fine)

                for ci, (w1, w2) in enumerate(fine_cands):
                    if self._stop_flag:
                        break
                    def _cb2(t, _tot, ci=ci, done=done):
                        pct = min(99, (done[0] + t) / max(1, total_sims) * 100)
                        self._after(lambda p=pct, c=ci, t_=t:
                            (self.prog_var.set(p),
                             self.prog_label.config(
                                 text=f"Phase 2 · Cand {c+1}/{len(fine_cands)} · Trial {t_}/{n_trials}")))
                    r = engine.run_for_weights(w1, w2, progress_cb=_cb2)
                    if r:
                        self.mc_results.append(r)
                    done[0] += n_trials

            if self.mc_results:
                self.best_result = max(self.mc_results, key=lambda r: r["mean_pdr"])
                self._after(self._on_optimizer_done)
            else:
                self._after(lambda: self._set_status("No valid results produced."))

        except Exception as exc:
            self._after(lambda: messagebox.showerror("Optimizer Error", str(exc)))

    def _on_optimizer_done(self):
        b = self.best_result
        self.best_label.config(
            text=(f"w1 = {b['w1']:.4f}   w2 = {b['w2']:.4f}\n"
                   f"PDR = {b['mean_pdr']:.4f}  ·  Energy = {b['mean_energy']:.2f}\n"
                   f"Collision = {b['mean_collision_rate']:.4f}  ·  Reach = {b['reach_rate']:.4f}"))
        self.prog_var.set(100)
        self.prog_label.config(text="Optimization complete ✅")
        self._set_status(
            f"Done · Best: w1={b['w1']:.4f}, w2={b['w2']:.4f}, PDR={b['mean_pdr']:.4f}")
        self._update_opt_chart()
        self._draw_results_charts()
        messagebox.showinfo(
            "Optimization Complete",
            f"Best weights found:\n"
            f"  w1 = {b['w1']:.4f}   w2 = {b['w2']:.4f}\n\n"
            f"  PDR              = {b['mean_pdr']:.4f}\n"
            f"  Reach Rate       = {b['reach_rate']:.4f}\n"
            f"  Energy           = {b['mean_energy']:.2f}\n"
            f"  Collision Rate   = {b['mean_collision_rate']:.4f}\n\n"
            f"Switch to 'Emergency Profiles' → click 'Generate Profiles'")

    def _stop_optimizer(self):
        self._stop_flag = True
        self._set_status("Stopping …")

    def _clear_results(self):
        if messagebox.askyesno("Clear?", "Clear all MC results and profiles?"):
            self.mc_results.clear()
            self.best_result = None
            self.profiler = EmergencyProfiler()
            self.prog_var.set(0)
            self.prog_label.config(text="Idle")
            self.best_label.config(text="Not yet optimized")
            self._draw_opt_chart_empty()
            self._set_status("Results cleared.")

    def _export_csv(self):
        if not self.mc_results:
            messagebox.showinfo("No Data", "Run the optimizer first.")
            return
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
            title="Save MC Results")
        if not path:
            return
        keys = ["w1", "w2", "mean_pdr", "mean_energy", "mean_collision_rate",
                "mean_channel_util", "mean_hop_count", "reach_rate",
                "mean_battery_remaining", "mean_rssi", "mean_neighbors", "n_valid"]
        with open(path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(self.mc_results)
        messagebox.showinfo("Exported", f"Results saved to:\n{path}")

    # ──────────────────────────────────────────────────────────────────
    # ACTIONS – EMERGENCY PROFILES
    # ──────────────────────────────────────────────────────────────────
    def _generate_profiles(self):
        if not HAVE_SKLEARN:
            messagebox.showerror("Missing Dependency",
                                   "Install scikit-learn:  pip install scikit-learn")
            return
        if len(self.mc_results) < 5:
            messagebox.showwarning("Insufficient Data",
                                    "Run the optimizer with more candidates/trials first.")
            return
        self.emerg_info.config(
            text="Training MLP on Monte Carlo dataset …  Please wait.",
            fg=self.WARN)
        self.mlp_info_label.config(text="Training …", fg=self.WARN)
        self.root.update()

        def _train():
            ok, msg = self.profiler.train(self.mc_results, n_clusters=3)
            self._after(lambda: self._on_profiles_done(ok, msg))

        threading.Thread(target=_train, daemon=True).start()

    def _on_profiles_done(self, ok: bool, msg: str):
        if not ok:
            self.emerg_info.config(text=f"❌ {msg}", fg=self.DANGER)
            self.mlp_info_label.config(text=f"Error: {msg}", fg=self.DANGER)
            return

        self.emerg_info.config(
            text=f"✅ {msg}  —  3 emergency weight profiles discovered from your simulation data.",
            fg=self.SUCCESS)
        self.mlp_info_label.config(
            text=(f"Architecture: MLPRegressor  ·  Hidden layers: (64, 64)  ·  "
                   f"Activation: ReLU  ·  Trained on {len(self.mc_results)} MC data points  ·  "
                   f"Features: [battery, RSSI, neighbors, collision, channel_util, PDR]  ·  "
                   f"Labels: [w1, w2]  ·  Profiles via K-Means (k=3)"),
            fg=self.FG_DIM)

        self._render_profile_cards()
        self._set_status("Profiles generated ✅")
        self._draw_results_charts()

    def _run_emergency_sim(self, profile: dict):
        self._run_vis_sim(profile["w1"], profile["w2"],
                           title=f"{profile['icon']} {profile['name']}")

    # ──────────────────────────────────────────────────────────────────
    # ACTIONS – NETWORK VIEW
    # ──────────────────────────────────────────────────────────────────
    def _run_best_sim(self):
        if not self.best_result:
            messagebox.showinfo("No Result", "Run the optimizer first.")
            return
        b = self.best_result
        self._run_vis_sim(b["w1"], b["w2"],
                           title=f"Best Weights  (w1={b['w1']:.4f}, w2={b['w2']:.4f})")

    def _run_vis_sim(self, w1: float, w2: float, title: str = ""):
        """Run one visualization simulation and display it in the Network View tab."""
        sf  = self.sf_var.get()
        n   = min(random.randint(self.nodes_min_var.get(), self.nodes_max_var.get()), 160)
        area = random.uniform(self.area_min_var.get(), self.area_max_var.get())
        batt = np.random.uniform(self.batt_min_var.get(), self.batt_max_var.get(), size=n)
        mode = random.choice(DEPLOY_MODES)
        mr   = NetworkGenerator.max_range_for_sf(sf)
        eff  = float(np.clip(mr, area * 0.12, area * 0.38))

        r = run_single_simulation(n, area, mode, batt, eff, w1, w2)
        if r is None:
            messagebox.showwarning("Sparse Network",
                                    "No valid path found – network may be too sparse.\n"
                                    "Try again (network is randomised each time).")
            return

        self.last_sim_result = r
        self.nb.select(self.tab_net)
        self._draw_network_tab(r, title=title)
        self._set_status(
            f"Sim: {title}  ·  Reached={r['reached']}  ·  PDR={r['pdr']:.3f}")

    # ──────────────────────────────────────────────────────────────────
    # HELPERS
    # ──────────────────────────────────────────────────────────────────
    def _set_status(self, text: str):
        self.root.after(0, lambda: self.status_var.set(text))

    def _after(self, fn):
        self.root.after(0, fn)

    def _btn(self, parent, text, cmd, bg, fg):
        return tk.Button(parent, text=text, command=cmd,
                          bg=bg, fg=fg,
                          font=("Segoe UI", 10, "bold"),
                          relief="flat", cursor="hand2",
                          activebackground=bg, activeforeground=fg,
                          pady=8, padx=10)

    def _spin(self, parent, var, lo, hi):
        return tk.Spinbox(parent, from_=lo, to=hi, textvariable=var, width=9,
                           bg=self.BG2, fg=self.FG,
                           buttonbackground=self.BG3,
                           insertbackground=self.ACCENT,
                           relief="flat", font=("Segoe UI", 9))

    def _entry(self, parent, var):
        return tk.Entry(parent, textvariable=var, width=10,
                         bg=self.BG2, fg=self.FG,
                         insertbackground=self.ACCENT,
                         relief="flat", font=("Segoe UI", 9))


# =====================================================================
# ENTRY POINT
# =====================================================================
def main():
    root = tk.Tk()
    OptimizerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
