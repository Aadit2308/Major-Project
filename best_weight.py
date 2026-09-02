"""
Weight Sweep for Hybrid Routing (w1 = RSSI weight, w2 = 1 - w1 = Battery weight)
==================================================================================
Python port of the original MATLAB script, rewritten for speed:

  - Pairwise distance / RSSI matrix built with vectorized numpy ops instead of
    a nested Python double-loop (this was the main MATLAB bottleneck).
  - Neighbor lookups done via boolean-masked numpy indexing.
  - The w1 sweep (101 independent points) is embarrassingly parallel, so it is
    farmed out across CPU cores with multiprocessing.Pool. Each worker still
    runs the same physical model / logic as the MATLAB version.

Goal: sweep w1 (RSSI weight) from 0 to 1 in steps of 0.01, w2 = 1 - w1
(battery weight), and find the value that gives the BEST BALANCED performance
across PDR (higher better), Network Lifetime (higher better), and End-to-end
Latency (lower better). "Balanced" = each metric contributes 1/3 to a
normalized composite score.
"""

import time
import numpy as np
import matplotlib.pyplot as plt
from multiprocessing import Pool, cpu_count

# =====================================================
# PARAMETERS  (same physical model as the main sim)
# =====================================================
AREA_SIZE = 1000
NETWORK_SIZES = [10, 50, 100]
SF_LIST = [7, 8, 9, 10, 11, 12]
SF_SENS = [-123, -126, -129, -132, -134, -137]
PT = 14
FREQ = 433
N_PL = 3.0
BATT_DEPL = 10
TX_COST = 2
LAMBDA_PKT = 0.002  # pure-ALOHA offered traffic (packets/sec/node)

AIRTIME_MS = {7: 56, 8: 103, 9: 185, 10: 370, 11: 660, 12: 1319}

# --- Sweep resolution & cost control ---
W1_VALUES = np.round(np.arange(0, 1.0001, 0.01), 2)  # 0:0.01:1 -> 101 points
NUM_TRIALS_SWEEP = 15  # trials per (w1, N, SF) combo
# total simulated trials = 101 * 3 * 6 * 15 = 27,270 (same as MATLAB version)

N_WORKERS = max(cpu_count() - 1, 1)


# =====================================================
# CORE PHYSICAL / ROUTING FUNCTIONS
# =====================================================
def compute_rssi(d, Pt, freq, n_pl):
    """Vectorized log-distance path-loss RSSI. Works on scalars or arrays."""
    d = np.maximum(d, 1)
    PL0 = 32.44 + 20 * np.log10(freq)
    PL = PL0 + 10 * n_pl * np.log10(d)
    return Pt - PL


def build_network(N, area_size, Pt, freq, n_pl, sens, rng):
    """Vectorized replacement for MATLAB's O(N^2) nested-for RSSI/neighbor build."""
    pos = area_size * rng.random((N, 2))
    diff = pos[:, None, :] - pos[None, :, :]
    dist = np.sqrt((diff ** 2).sum(axis=-1))
    rssi_mat = compute_rssi(dist, Pt, freq, n_pl)
    np.fill_diagonal(rssi_mat, -np.inf)
    adj = rssi_mat >= sens
    np.fill_diagonal(adj, False)
    neighbors = [np.where(adj[i])[0] for i in range(N)]
    return rssi_mat, neighbors, adj


def hybrid_greedy_route(src, dst, neighbors, rssi_mat, battery, w1, w2, N):
    visited = np.zeros(N, dtype=bool)
    current = src
    visited[current] = True
    path = [current]

    while current != dst:
        nbrs = neighbors[current]
        nbrs = nbrs[~visited[nbrs]]

        if nbrs.size == 0:
            return None, np.inf

        rssi_vals = rssi_mat[current, nbrs]
        batt_vals = battery[nbrs]

        if w2 > 0:
            r_min, r_max = rssi_vals.min(), rssi_vals.max()
            b_min, b_max = batt_vals.min(), batt_vals.max()

            rssi_norm = (np.ones_like(rssi_vals) if r_max == r_min
                         else (rssi_vals - r_min) / (r_max - r_min))
            batt_norm = (np.ones_like(batt_vals) if b_max == b_min
                         else (batt_vals - b_min) / (b_max - b_min))

            scores = w1 * rssi_norm + w2 * batt_norm
        else:
            scores = rssi_vals

        idx = int(np.argmax(scores))
        nxt = int(nbrs[idx])
        path.append(nxt)
        visited[nxt] = True
        current = nxt

    return np.array(path), len(path) - 1


def aloha_collision_prob(num_interferers, lambda_pkt, ton_sec):
    lambda_total = max(num_interferers, 0) * lambda_pkt
    return 1 - np.exp(-2 * lambda_total * ton_sec)


def path_collision_survival(path, neighbors, lambda_pkt, ton_sec):
    if path is None or len(path) < 2:
        return 1.0, 0.0
    p_success = 1.0
    pc_vals = []
    for k in range(1, len(path)):
        rx = path[k]
        num_interf = max(len(neighbors[rx]) - 1, 0)
        pc_hop = aloha_collision_prob(num_interf, lambda_pkt, ton_sec)
        pc_vals.append(pc_hop)
        p_success *= (1 - pc_hop)
    return p_success, float(np.mean(pc_vals))


def evaluate_weights(w1, w2, network_sizes, sf_list, sf_sens, num_trials,
                      area_size, Pt, freq, n_pl, batt_depl, tx_cost,
                      lambda_pkt, airtime_ms, seed):
    rng = np.random.default_rng(seed)

    pdr_acc = 0.0
    lifetime_acc = 0.0
    latency_acc = 0.0
    latency_n = 0
    scenario_count = len(network_sizes) * len(sf_list)

    for N in network_sizes:
        for sf, sens in zip(sf_list, sf_sens):
            ton_sec = airtime_ms[sf] / 1000.0

            pdr_sum = 0
            lifetime_sum = 0
            lat_sum = 0.0
            lat_count = 0

            for _ in range(num_trials):
                rssi_mat, neighbors, _ = build_network(
                    N, area_size, Pt, freq, n_pl, sens, rng)
                battery = 20 + 80 * rng.random(N)

                # --- PDR + Latency (collision-aware) ---
                node_perm = rng.permutation(N)
                src, dst = int(node_perm[0]), int(node_perm[1])
                batt_copy = battery.copy()
                path, hops = hybrid_greedy_route(
                    src, dst, neighbors, rssi_mat, batt_copy, w1, w2, N)

                if path is not None:
                    p_success, _ = path_collision_survival(
                        path, neighbors, lambda_pkt, ton_sec)
                    if rng.random() <= p_success:
                        pdr_sum += 1
                        lat_sum += hops * airtime_ms[sf]
                        lat_count += 1

                # --- Network Lifetime ---
                batt_life = battery.copy()
                round_count = 0
                alive = True
                while alive:
                    active = np.where(batt_life > batt_depl)[0]
                    if active.size < 2:
                        break

                    active_mask = np.zeros(N, dtype=bool)
                    active_mask[active] = True
                    nbrs_live = [np.array([], dtype=int)] * N
                    for u in active:
                        row = rssi_mat[u] >= sens
                        row &= active_mask
                        row[u] = False
                        nbrs_live[u] = np.where(row)[0]

                    rp = rng.choice(active, size=2, replace=False)
                    s, d_ = int(rp[0]), int(rp[1])
                    p, _ = hybrid_greedy_route(
                        s, d_, nbrs_live, rssi_mat, batt_life, w1, w2, N)
                    if p is None:
                        break

                    relays = p[1:-1]
                    if relays.size > 0:
                        batt_life[relays] -= tx_cost
                    round_count += 1

                    if np.any(batt_life[active] <= batt_depl):
                        alive = False

                lifetime_sum += round_count

            pdr_acc += (pdr_sum / num_trials) * 100
            lifetime_acc += (lifetime_sum / num_trials)
            if lat_count > 0:
                latency_acc += (lat_sum / lat_count)
                latency_n += 1

    pdr_pct = pdr_acc / scenario_count
    lifetime_rounds = lifetime_acc / scenario_count
    latency_ms = (latency_acc / latency_n) if latency_n > 0 else 0.0

    return pdr_pct, lifetime_rounds, latency_ms


def _worker(args):
    k, w1 = args
    w2 = round(1 - w1, 2)
    pdr, life, lat = evaluate_weights(
        w1, w2, NETWORK_SIZES, SF_LIST, SF_SENS, NUM_TRIALS_SWEEP,
        AREA_SIZE, PT, FREQ, N_PL, BATT_DEPL, TX_COST, LAMBDA_PKT,
        AIRTIME_MS, seed=1000 + k)
    return k, w1, w2, pdr, life, lat


# =====================================================
# MAIN
# =====================================================
def main():
    total_trials = len(W1_VALUES) * len(NETWORK_SIZES) * len(SF_LIST) * NUM_TRIALS_SWEEP
    print(f"Sweeping {len(W1_VALUES)} weight combinations "
          f"({total_trials} total simulated trials) using {N_WORKERS} workers...")

    PDR_sweep = np.zeros(len(W1_VALUES))
    Lifetime_sweep = np.zeros(len(W1_VALUES))
    Latency_sweep = np.zeros(len(W1_VALUES))

    tasks = list(enumerate(W1_VALUES))
    t_start = time.time()
    done = 0

    with Pool(processes=N_WORKERS) as pool:
        for k, w1, w2, pdr, life, lat in pool.imap_unordered(_worker, tasks):
            PDR_sweep[k] = pdr
            Lifetime_sweep[k] = life
            Latency_sweep[k] = lat
            done += 1
            if done % 10 == 0 or done == len(W1_VALUES):
                print(f"  [{done:3d}/{len(W1_VALUES)}] latest: w1={w1:.2f} w2={w2:.2f} "
                      f"PDR={pdr:.1f}% Life={life:.1f} Lat={lat:.1f}ms "
                      f"({time.time()-t_start:.0f}s elapsed)")

    # =====================================================
    # NORMALIZE + COMPOSITE (BALANCED) SCORE
    # =====================================================
    def norm01(x):
        x = np.asarray(x, dtype=float)
        return (x - x.min()) / (x.max() - x.min() + np.finfo(float).eps)

    PDR_norm = norm01(PDR_sweep)
    Lifetime_norm = norm01(Lifetime_sweep)
    Latency_norm = 1 - norm01(Latency_sweep)  # lower latency is better -> invert

    Composite_score = (PDR_norm + Lifetime_norm + Latency_norm) / 3

    best_idx = int(np.argmax(Composite_score))
    best_score = Composite_score[best_idx]
    best_w1 = round(float(W1_VALUES[best_idx]), 2)
    best_w2 = round(1 - best_w1, 2)

    print("\n=================================================================")
    print(" BEST BALANCED WEIGHTS FOUND")
    print("=================================================================")
    print(f" w1 (RSSI weight)    = {best_w1:.2f}")
    print(f" w2 (Battery weight) = {best_w2:.2f}")
    print(f" Composite score     = {best_score:.4f}  (0-1 scale, higher = better balance)")
    print(f" At this point: PDR = {PDR_sweep[best_idx]:.1f}%, "
          f"Lifetime = {Lifetime_sweep[best_idx]:.1f} rounds, "
          f"Latency = {Latency_sweep[best_idx]:.1f} ms")
    print("=================================================================")

    # =====================================================
    # PLOTTING
    # =====================================================
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    fig.suptitle("Weight Sweep for Hybrid Routing (w1 = RSSI, w2 = 1-w1 = Battery)",
                  fontsize=13)

    ax = axes[0, 0]
    ax.plot(W1_VALUES, PDR_sweep, '-', color=(0.18, 0.44, 0.71), linewidth=1.8)
    ax.axvline(best_w1, color='r', linestyle='--', linewidth=1.5)
    ax.set_xlabel('w1 (RSSI weight)'); ax.set_ylabel('PDR (%)')
    ax.set_title('PDR vs w1'); ax.grid(True)

    ax = axes[0, 1]
    ax.plot(W1_VALUES, Lifetime_sweep, '-', color=(0.18, 0.55, 0.34), linewidth=1.8)
    ax.axvline(best_w1, color='r', linestyle='--', linewidth=1.5)
    ax.set_xlabel('w1 (RSSI weight)'); ax.set_ylabel('Network Lifetime (rounds)')
    ax.set_title('Lifetime vs w1'); ax.grid(True)

    ax = axes[1, 0]
    ax.plot(W1_VALUES, Latency_sweep, '-', color=(0.84, 0.27, 0.27), linewidth=1.8)
    ax.axvline(best_w1, color='r', linestyle='--', linewidth=1.5)
    ax.set_xlabel('w1 (RSSI weight)'); ax.set_ylabel('Avg Latency (ms)')
    ax.set_title('Latency vs w1'); ax.grid(True)

    ax = axes[1, 1]
    ax.plot(W1_VALUES, Composite_score, '-', color=(0.49, 0.18, 0.56), linewidth=2.2)
    ax.plot(best_w1, best_score, 'p', markersize=16,
            markerfacecolor=(1, 0.65, 0), markeredgecolor='k')
    ax.axvline(best_w1, color='r', linestyle='--', linewidth=1.5)
    ax.set_xlabel('w1 (RSSI weight)'); ax.set_ylabel('Balanced Composite Score')
    ax.set_title(f'Composite Score vs w1  (best: w1={best_w1:.2f}, w2={best_w2:.2f})')
    ax.grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_path = "weight_sweep_results.png"
    plt.savefig(out_path, dpi=150)
    print(f"\nSaved plot to {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
