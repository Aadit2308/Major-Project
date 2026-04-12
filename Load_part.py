import tkinter as tk
from tkinter import ttk, font
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.patches import FancyArrowPatch
import matplotlib.animation as animation
import math
import time
import threading
import random
from collections import defaultdict

# =====================================================
# LoRa PHY Parameters
# =====================================================
SF_SENS = {7: -123, 8: -126, 9: -129, 10: -132, 11: -134, 12: -137}

NODE_COLORS = ["#FF6B6B", "#4ECDC4", "#45B7D1", "#96CEB4", "#FFEAA7"]
NODE_NAMES  = ["Node 0", "Node 1", "Node 2", "Node 3", "Node 4"]

def compute_rssi(d, Pt=14, freq=433e6, n=3.0):
    if d < 1:
        d = 1
    PL0 = 32.44 + 20 * math.log10(freq / 1e6)
    PL  = PL0 + 10 * n * math.log10(d)
    return Pt - PL

# =====================================================
# Routing Algorithms
# =====================================================
ALGO_GREEDY_NEIGHBOR = "Greedy Neighbor (RSSI Only)"
ALGO_CUSTOM          = "Custom Greedy (RSSI + Battery)"

def greedy_neighbor(src, neighbors, rssi, visited):
    scores = {}
    for nb in neighbors[src]:
        if nb not in visited:
            scores[nb] = rssi.get((src, nb), -999)
    if not scores:
        return None, scores
    return max(scores, key=scores.get), scores

def custom_greedy(src, neighbors, rssi, battery, visited, w1=0.6, w2=0.4):
    scores = {}
    for nb in neighbors[src]:
        if nb not in visited:
            r  = rssi.get((src, nb), -999)
            b  = battery.get(nb, 0)
            scores[nb] = w1 * r + w2 * b
    if not scores:
        return None, scores
    return max(scores, key=scores.get), scores

# =====================================================
# Main Application
# =====================================================
class LoRaSimApp:
    def __init__(self, root):
        self.root = root
        self.root.title("LoRa Simultaneous Packet Transmission Simulator")
        self.root.configure(bg="#0D1117")
        self.root.geometry("1400x900")
        self.root.minsize(1200, 800)

        self.nodes  = list(range(5))
        self.edges  = [(i, j) for i in self.nodes for j in self.nodes if i < j]
        self.pos    = {
            0: (0.0,  1.0),
            1: (0.95, 0.31),
            2: (0.59, -0.81),
            3: (-0.59, -0.81),
            4: (-0.95, 0.31),
        }

        self.sf_var      = tk.IntVar(value=9)
        self.algo_var    = tk.StringVar(value=ALGO_CUSTOM)
        self.w1_var      = tk.DoubleVar(value=0.6)
        self.w2_var      = tk.DoubleVar(value=0.4)
        self.battery     = {n: tk.IntVar(value=80) for n in self.nodes}
        self.dist_vars   = {(u, v): tk.DoubleVar(value=120.0) for (u, v) in self.edges}

        self.sim_running = False
        self.packet_threads = []
        self.log_lock    = threading.Lock()

        self._build_ui()

    # --------------------------------------------------
    # UI Construction
    # --------------------------------------------------
    def _build_ui(self):
        # ── Top banner ──────────────────────────────────
        banner = tk.Frame(self.root, bg="#161B22", height=56)
        banner.pack(fill="x")
        tk.Label(banner, text="⬡  LoRa Simultaneous Transmission Simulator",
                 bg="#161B22", fg="#58A6FF",
                 font=("Courier New", 17, "bold")).pack(side="left", padx=18, pady=10)
        tk.Label(banner, text="5-Node Network · Multi-Hop · Real-Time",
                 bg="#161B22", fg="#8B949E",
                 font=("Courier New", 10)).pack(side="right", padx=18)

        # ── Main panes ──────────────────────────────────
        pane = tk.PanedWindow(self.root, orient="horizontal",
                              bg="#0D1117", sashwidth=6, sashrelief="flat")
        pane.pack(fill="both", expand=True, padx=6, pady=6)

        left  = tk.Frame(pane, bg="#0D1117")
        right = tk.Frame(pane, bg="#0D1117")
        pane.add(left,  minsize=320)
        pane.add(right, minsize=700)

        self._build_control_panel(left)
        self._build_graph_panel(right)

    def _build_control_panel(self, parent):
        canvas = tk.Canvas(parent, bg="#0D1117", highlightthickness=0)
        sb     = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scroll_frame = tk.Frame(canvas, bg="#0D1117")
        scroll_frame.bind("<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=sb.set)
        canvas.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")

        pad = {"padx": 10, "pady": 4}

        # ── Algorithm ────────────────────────────────────
        self._section(scroll_frame, "ALGORITHM")
        for algo in [ALGO_GREEDY_NEIGHBOR, ALGO_CUSTOM]:
            tk.Radiobutton(scroll_frame, text=algo, variable=self.algo_var,
                           value=algo, bg="#0D1117", fg="#C9D1D9",
                           selectcolor="#21262D", activebackground="#0D1117",
                           font=("Courier New", 9),
                           command=self._toggle_weights).pack(anchor="w", **pad)

        # ── Weights ──────────────────────────────────────
        self._section(scroll_frame, "WEIGHTS  (w1 + w2 = 1.0)")
        wf = tk.Frame(scroll_frame, bg="#0D1117")
        wf.pack(fill="x", padx=10)

        self.w1_lbl = tk.Label(wf, text="w1 (RSSI): 0.60",
                               bg="#0D1117", fg="#58A6FF",
                               font=("Courier New", 9, "bold"))
        self.w1_lbl.grid(row=0, column=0, sticky="w")
        self.w1_scale = tk.Scale(wf, from_=0, to=1, resolution=0.05,
                                 orient="horizontal", variable=self.w1_var,
                                 bg="#0D1117", fg="#58A6FF",
                                 troughcolor="#21262D", highlightthickness=0,
                                 command=self._sync_weights, length=200)
        self.w1_scale.grid(row=1, column=0, sticky="ew")

        self.w2_lbl = tk.Label(wf, text="w2 (Battery): 0.40",
                               bg="#0D1117", fg="#96CEB4",
                               font=("Courier New", 9, "bold"))
        self.w2_lbl.grid(row=2, column=0, sticky="w")
        self.w2_scale = tk.Scale(wf, from_=0, to=1, resolution=0.05,
                                 orient="horizontal", variable=self.w2_var,
                                 bg="#0D1117", fg="#96CEB4",
                                 troughcolor="#21262D", highlightthickness=0,
                                 command=self._sync_weights_from_w2, length=200)
        self.w2_scale.grid(row=3, column=0, sticky="ew")

        # ── Spreading Factor ─────────────────────────────
        self._section(scroll_frame, "SPREADING FACTOR (SF)")
        sf_f = tk.Frame(scroll_frame, bg="#0D1117")
        sf_f.pack(fill="x", padx=10)
        for sf in SF_SENS:
            tk.Radiobutton(sf_f, text=f"SF{sf}  (sens {SF_SENS[sf]} dBm)",
                           variable=self.sf_var, value=sf,
                           bg="#0D1117", fg="#C9D1D9",
                           selectcolor="#21262D", activebackground="#0D1117",
                           font=("Courier New", 8)).pack(anchor="w")

        # ── Battery ──────────────────────────────────────
        self._section(scroll_frame, "BATTERY (%)")
        for n in self.nodes:
            row = tk.Frame(scroll_frame, bg="#0D1117")
            row.pack(fill="x", padx=10, pady=2)
            tk.Label(row, text=f"Node {n}", bg="#0D1117",
                     fg=NODE_COLORS[n], font=("Courier New", 9, "bold"),
                     width=8).pack(side="left")
            tk.Scale(row, from_=0, to=100, orient="horizontal",
                     variable=self.battery[n],
                     bg="#0D1117", fg=NODE_COLORS[n],
                     troughcolor="#21262D", highlightthickness=0,
                     length=160).pack(side="left")
            tk.Label(row, textvariable=self.battery[n],
                     bg="#0D1117", fg="#8B949E",
                     font=("Courier New", 8), width=4).pack(side="left")

        # ── Distances ────────────────────────────────────
        self._section(scroll_frame, "EDGE DISTANCES (m)")
        for (u, v) in self.edges:
            row = tk.Frame(scroll_frame, bg="#0D1117")
            row.pack(fill="x", padx=10, pady=1)
            tk.Label(row, text=f"{u}↔{v}", bg="#0D1117", fg="#8B949E",
                     font=("Courier New", 8), width=5).pack(side="left")
            tk.Scale(row, from_=50, to=1000, orient="horizontal",
                     variable=self.dist_vars[(u, v)],
                     bg="#0D1117", fg="#E6EDF3",
                     troughcolor="#21262D", highlightthickness=0,
                     length=140).pack(side="left")
            lbl = tk.Label(row, bg="#0D1117", fg="#8B949E",
                           font=("Courier New", 8), width=5)
            lbl.pack(side="left")
            self.dist_vars[(u, v)].trace_add("write",
                lambda *a, lbl=lbl, v=self.dist_vars[(u, v)]: lbl.config(
                    text=f"{int(v.get())}m"))
            lbl.config(text=f"{int(self.dist_vars[(u,v)].get())}m")

        # ── Buttons ──────────────────────────────────────
        btn_f = tk.Frame(scroll_frame, bg="#0D1117")
        btn_f.pack(fill="x", padx=10, pady=10)

        self.run_btn = tk.Button(btn_f, text="▶  START SIMULATION",
                                 bg="#238636", fg="white",
                                 font=("Courier New", 10, "bold"),
                                 relief="flat", cursor="hand2",
                                 command=self.start_simulation)
        self.run_btn.pack(fill="x", pady=3)

        tk.Button(btn_f, text="↺  RESET",
                  bg="#21262D", fg="#C9D1D9",
                  font=("Courier New", 9),
                  relief="flat", cursor="hand2",
                  command=self.reset).pack(fill="x", pady=3)

        self._toggle_weights()

    def _build_graph_panel(self, parent):
        # ── Graph canvas ─────────────────────────────────
        self.fig, self.ax = plt.subplots(figsize=(6, 5))
        self.fig.patch.set_facecolor("#0D1117")
        self.ax.set_facecolor("#0D1117")

        self.canvas_widget = FigureCanvasTkAgg(self.fig, parent)
        self.canvas_widget.get_tk_widget().pack(fill="both", expand=True,
                                                padx=4, pady=4)

        # ── Log panel ────────────────────────────────────
        log_frame = tk.Frame(parent, bg="#161B22")
        log_frame.pack(fill="x", padx=4, pady=(0, 4))

        tk.Label(log_frame, text="TRANSMISSION LOG",
                 bg="#161B22", fg="#58A6FF",
                 font=("Courier New", 9, "bold")).pack(anchor="w", padx=8, pady=4)

        self.log_box = tk.Text(log_frame, height=10, bg="#0D1117",
                               fg="#C9D1D9", font=("Courier New", 8),
                               insertbackground="white", relief="flat",
                               state="disabled", wrap="word")
        self.log_box.pack(fill="x", padx=6, pady=(0, 6))

        # colour tags
        for node, col in enumerate(NODE_COLORS):
            self.log_box.tag_config(f"n{node}", foreground=col)
        self.log_box.tag_config("success", foreground="#3FB950")
        self.log_box.tag_config("fail",    foreground="#F85149")
        self.log_box.tag_config("info",    foreground="#8B949E")
        self.log_box.tag_config("header",  foreground="#58A6FF")
        self.log_box.tag_config("weight",  foreground="#D2A8FF")

        self._draw_graph()

    # --------------------------------------------------
    # Helpers
    # --------------------------------------------------
    def _section(self, parent, title):
        tk.Label(parent, text=f"── {title} ──",
                 bg="#0D1117", fg="#58A6FF",
                 font=("Courier New", 8, "bold")).pack(anchor="w",
                                                       padx=10, pady=(10, 2))

    def _toggle_weights(self):
        state = "normal" if self.algo_var.get() == ALGO_CUSTOM else "disabled"
        self.w1_scale.config(state=state)
        self.w2_scale.config(state=state)

    def _sync_weights(self, val=None):
        w1 = round(self.w1_var.get(), 2)
        w2 = round(1.0 - w1, 2)
        self.w2_var.set(w2)
        self.w1_lbl.config(text=f"w1 (RSSI): {w1:.2f}")
        self.w2_lbl.config(text=f"w2 (Battery): {w2:.2f}")

    def _sync_weights_from_w2(self, val=None):
        w2 = round(self.w2_var.get(), 2)
        w1 = round(1.0 - w2, 2)
        self.w1_var.set(w1)
        self.w1_lbl.config(text=f"w1 (RSSI): {w1:.2f}")
        self.w2_lbl.config(text=f"w2 (Battery): {w2:.2f}")

    def _build_network(self):
        sf   = self.sf_var.get()
        sens = SF_SENS[sf]

        G         = nx.Graph()
        rssi      = {}
        neighbors = {n: set() for n in self.nodes}

        G.add_nodes_from(self.nodes)
        for (u, v), dvar in self.dist_vars.items():
            d = dvar.get()
            r = compute_rssi(d)
            if r >= sens:
                G.add_edge(u, v)
                rssi[(u, v)] = rssi[(v, u)] = r
                neighbors[u].add(v)
                neighbors[v].add(u)

        return G, rssi, neighbors

    # --------------------------------------------------
    # Graph Drawing
    # --------------------------------------------------
    def _draw_graph(self, paths=None):
        """Draw the network; paths = {src: list_of_nodes}."""
        self.ax.clear()
        self.ax.set_facecolor("#0D1117")

        G, rssi, _ = self._build_network()

        # Node colours
        node_colors = [NODE_COLORS[n] for n in G.nodes]

        nx.draw_networkx_nodes(G, self.pos, ax=self.ax,
                               node_size=1600, node_color=node_colors,
                               alpha=0.92)

        nx.draw_networkx_edges(G, self.pos, ax=self.ax,
                               edge_color="#30363D", width=2)

        # Labels with battery %
        labels = {n: f"N{n}\n{self.battery[n].get()}%" for n in G.nodes}
        nx.draw_networkx_labels(G, self.pos, labels, ax=self.ax,
                                font_size=8, font_color="#0D1117",
                                font_weight="bold")

        # RSSI labels
        edge_labels = {(u, v): f"{int(rssi[(u,v)])} dBm"
                       for (u, v) in G.edges if (u, v) in rssi}
        nx.draw_networkx_edge_labels(G, self.pos, edge_labels,
                                     ax=self.ax, font_size=7,
                                     font_color="#8B949E")

        # Draw active paths
        if paths:
            for src, path in paths.items():
                if len(path) > 1:
                    path_edges = list(zip(path, path[1:]))
                    nx.draw_networkx_edges(G, self.pos, ax=self.ax,
                                          edgelist=path_edges,
                                          edge_color=NODE_COLORS[src],
                                          width=3.5, alpha=0.9,
                                          style="dashed")

        self.ax.set_title("LoRa Network – Simultaneous Transmission",
                          color="#C9D1D9", fontsize=10,
                          fontfamily="monospace", pad=10)
        self.ax.axis("off")
        self.canvas_widget.draw()

    # --------------------------------------------------
    # Log helpers (thread-safe)
    # --------------------------------------------------
    def _log(self, text, tag="info"):
        def _do():
            self.log_box.config(state="normal")
            self.log_box.insert("end", text + "\n", tag)
            self.log_box.see("end")
            self.log_box.config(state="disabled")
        self.root.after(0, _do)

    def _clear_log(self):
        self.log_box.config(state="normal")
        self.log_box.delete("1.0", "end")
        self.log_box.config(state="disabled")

    # --------------------------------------------------
    # Simulation
    # --------------------------------------------------
    def start_simulation(self):
        if self.sim_running:
            return
        self.sim_running = True
        self.run_btn.config(state="disabled", text="⏳  RUNNING…")
        self._clear_log()

        G, rssi, neighbors = self._build_network()
        batt  = {n: self.battery[n].get() for n in self.nodes}
        algo  = self.algo_var.get()
        w1    = round(self.w1_var.get(), 2)
        w2    = round(self.w2_var.get(), 2)

        self._log(f"═══════════════════════════════════════", "header")
        self._log(f"  SIMULTANEOUS PACKET TRANSMISSION", "header")
        self._log(f"  Algorithm : {algo}", "header")
        if algo == ALGO_CUSTOM:
            self._log(f"  Weights   : w1(RSSI)={w1}  w2(Battery)={w2}", "weight")
        self._log(f"  SF        : SF{self.sf_var.get()}", "header")
        self._log(f"  Nodes     : {len(self.nodes)}  (each transmits to all others)", "header")
        self._log(f"═══════════════════════════════════════", "header")

        # Each node transmits to every other node simultaneously
        self.active_paths = {n: [n] for n in self.nodes}
        results = {n: {} for n in self.nodes}  # results[src][dst] = path/None

        def run_one_packet(src, dst, algo, rssi, neighbors, batt, w1, w2):
            """Route a single packet from src to dst."""
            path    = [src]
            current = src
            visited = {src}
            success = False

            while current != dst:
                if algo == ALGO_GREEDY_NEIGHBOR:
                    nxt, scores = greedy_neighbor(current, neighbors, rssi, visited)
                else:
                    nxt, scores = custom_greedy(current, neighbors, rssi, batt, visited, w1, w2)

                if nxt is None:
                    break

                # Log the hop decision
                score_str = "  ".join(
                    [f"N{k}={round(v,1)}" for k, v in scores.items()])
                tag = f"n{src}"
                self._log(
                    f"  [N{src}→N{dst}]  @ N{current}  scores=[{score_str}]  → N{nxt}",
                    tag)

                path.append(nxt)
                visited.add(nxt)
                current = nxt
                time.sleep(0.25)

            if current == dst:
                success = True
                self._log(f"  ✓ N{src}→N{dst}  Path: {' → '.join(map(str,path))}", "success")
            else:
                self._log(f"  ✗ N{src}→N{dst}  FAILED  (stuck at N{current})", "fail")

            return path if success else None

        def simulate_all():
            threads = []
            path_lock  = threading.Lock()
            draw_paths = {}

            def worker(src, dst):
                path = run_one_packet(src, dst, algo, rssi, neighbors, batt, w1, w2)
                with path_lock:
                    if path:
                        # Merge into draw_paths
                        draw_paths[src] = draw_paths.get(src, [src])
                        # Append unique new hops
                        for node in path[1:]:
                            if node not in draw_paths[src]:
                                draw_paths[src].append(node)
                    results[src][dst] = path
                # Redraw after each packet settles
                self.root.after(0, lambda: self._draw_graph(draw_paths))

            # Launch all src→dst pairs simultaneously
            for src in self.nodes:
                for dst in self.nodes:
                    if src != dst:
                        t = threading.Thread(target=worker, args=(src, dst),
                                             daemon=True)
                        threads.append(t)

            for t in threads:
                t.start()
            for t in threads:
                t.join()

            # Summary
            self._log(f"\n═══════  SUMMARY  ═══════", "header")
            total = success = 0
            for src in self.nodes:
                for dst in self.nodes:
                    if src == dst:
                        continue
                    total += 1
                    if results[src][dst]:
                        success += 1
            self._log(f"  Delivered : {success}/{total} packets", "success")
            self._log(f"  Lost      : {total-success}/{total} packets", "fail" if total-success else "success")

            self.root.after(0, self._sim_done)

        threading.Thread(target=simulate_all, daemon=True).start()

    def _sim_done(self):
        self.sim_running = False
        self.run_btn.config(state="normal", text="▶  START SIMULATION")

    def reset(self):
        self.sim_running = False
        self.run_btn.config(state="normal", text="▶  START SIMULATION")
        self._clear_log()
        self._draw_graph()


# =====================================================
# Entry Point
# =====================================================
if __name__ == "__main__":
    root = tk.Tk()

    style = ttk.Style(root)
    style.theme_use("clam")
    style.configure("Vertical.TScrollbar",
                    background="#21262D", troughcolor="#0D1117",
                    bordercolor="#0D1117", arrowcolor="#8B949E")

    app = LoRaSimApp(root)
    root.mainloop()
