import tkinter as tk
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import time

# =====================================================
# LoRa PHY(this is of sfread factor values and their roesponding rssi values and from the below for rssi is calc based on path loss
# =====================================================
SF_SENS = {7:-123, 8:-126, 9:-129, 10:-132, 11:-134, 12:-137}

def compute_rssi(d, Pt=14, freq=433e6, n=3.0):
    if d < 1:
        d = 1
    PL0 = 32.44 + 20 * math.log10(freq/1e6)
    PL = PL0 + 10 * n * math.log10(d)
    return Pt - PL

# =====================================================
# Algorithm Labels
# =====================================================
ALGO_GREEDY_NEIGHBOR = "Greedy Neighbor Routing"
ALGO_CUSTOM = "Custom Greedy Algorithm"

# =====================================================
# Algorithms (FIXED: exclude visited) here the word src is source node 
# =====================================================
def greedy_neighbor(src, neighbors, rssi, visited):
    scores = {}
    for n in neighbors[src]:
        if n not in visited:
            scores[n] = rssi[(src, n)]
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores
##w1 and w2 are the   
def custom_greedy(src, neighbors, rssi, battery, visited, w1=0.6, w2=0.4):
    scores = {}
    for n in neighbors[src]:
        if n not in visited:
            score = w1*rssi[(src,n)] + w2*battery[n]
            scores[n] = score
    if not scores:
        return None, scores
    best = max(scores, key=scores.get)
    return best, scores

# =====================================================
# PARAMETER INPUT WINDOW this part set ups the gui thing
# =====================================================
class ParamInput:
    def __init__(self, root, nodes, edges):
        self.top = tk.Toplevel(root)
        self.top.title("Simulation Parameters")

        # SF Selection
        self.sf = tk.IntVar(value=9)
        tk.Label(self.top, text="Spreading Factor (SF)").grid(row=0,column=0)
        tk.OptionMenu(self.top, self.sf, *SF_SENS.keys()).grid(row=0,column=1)

        # Battery Input
        self.battery = {}
        tk.Label(self.top, text="Battery (%)").grid(row=1,column=0)
        for i,n in enumerate(nodes):
            e = tk.Entry(self.top,width=5)
            e.insert(0,"80")
            e.grid(row=2+i,column=1)
            tk.Label(self.top,text=f"Node {n}").grid(row=2+i,column=0)
            self.battery[n] = e

        # Distance Input
        self.dist = {}
        tk.Label(self.top, text="Distance (m)").grid(row=1,column=2)
        for i,(u,v) in enumerate(edges):
            e = tk.Entry(self.top,width=6)
            e.insert(0,"100")
            e.grid(row=2+i,column=3)
            tk.Label(self.top,text=f"{u}-{v}").grid(row=2+i,column=2)
            self.dist[(u,v)] = e

        # Source & Destination
        self.src = tk.IntVar(value=0)
        self.dst = tk.IntVar(value=1)

        tk.Label(self.top,text="Source").grid(row=8,column=0)
        tk.Label(self.top,text="Destination").grid(row=8,column=2)

        tk.OptionMenu(self.top,self.src,*nodes).grid(row=8,column=1)
        tk.OptionMenu(self.top,self.dst,*nodes).grid(row=8,column=3)

        tk.Button(self.top,text="Start",command=self.submit)\
            .grid(row=9,column=1,columnspan=2)

    def submit(self):
        self.batt_vals = {n:int(e.get()) for n,e in self.battery.items()}
        self.dist_vals = {k:float(e.get()) for k,e in self.dist.items()}
        self.sf_val = self.sf.get()
        self.source = self.src.get()
        self.dest = self.dst.get()
        self.top.destroy()

# =====================================================
# SIMULATOR WINDOW
# =====================================================
class Simulator:
    def __init__(self, root, algo, params):
        self.root = root
        self.algo = algo
        self.battery = params.batt_vals
        self.sf = params.sf_val
        self.src = params.source
        self.dst = params.dest

        self.root.title("LoRa Multi-Hop Simulation")

        # Node positions
        self.pos = {
            0:(0,1), 1:(0.95,0.31), 2:(0.59,-0.81),
            3:(-0.59,-0.81), 4:(-0.95,0.31)
        }

        self.G = nx.Graph()
        self.G.add_nodes_from(self.pos.keys())

        self.rssi = {}
        self.neighbors = {n:set() for n in self.G.nodes}
        sens = SF_SENS[self.sf]

        # Build edges based on RSSI threshold
        for (u,v),d in params.dist_vals.items():
            r = compute_rssi(d)
            if r >= sens:
                self.G.add_edge(u,v)
                self.rssi[(u,v)] = self.rssi[(v,u)] = r
                self.neighbors[u].add(v)
                self.neighbors[v].add(u)

        # Graph
        self.fig,self.ax = plt.subplots(figsize=(5,5))
        self.canvas = FigureCanvasTkAgg(self.fig,root)
        self.canvas.get_tk_widget().pack()

        # Score display
        self.score_box = tk.Text(root, height=8, width=70)
        self.score_box.pack()

        tk.Button(root,text="Run",command=self.run).pack()

        self.draw()

    def draw(self, path=None):
        self.ax.clear()
        labels={n:f"{n}\n{self.battery[n]}%" for n in self.G.nodes}

        nx.draw(self.G,self.pos,ax=self.ax,
                node_size=1400,node_color="lightblue",
                edge_color="gray",with_labels=False)
        nx.draw_networkx_labels(self.G,self.pos,labels,ax=self.ax)

        edge_labels={(u,v):f"{int(self.rssi[(u,v)])} dBm"
                     for (u,v) in self.G.edges}
        nx.draw_networkx_edge_labels(self.G,self.pos,
                                     edge_labels=edge_labels,
                                     font_size=8)

        if path:
            nx.draw_networkx_edges(self.G,self.pos,
                                   edgelist=list(zip(path,path[1:])),
                                   edge_color="red",width=3)

        self.canvas.draw()

    def run(self):
        path = [self.src]
        current = self.src
        visited = set([current])

        self.score_box.delete("1.0", tk.END)

        while current != self.dst:

            if self.algo == ALGO_GREEDY_NEIGHBOR:
                nxt, scores = greedy_neighbor(current, self.neighbors,
                                              self.rssi, visited)
            else:
                nxt, scores = custom_greedy(current, self.neighbors,
                                            self.rssi, self.battery,
                                            visited)

            if not scores:
                self.score_box.insert(tk.END,
                    f"\nNode {current} has no unvisited neighbors.\n")
                break

            self.score_box.insert(tk.END,
                f"\nCurrent Node: {current}\n")

            for node, score in scores.items():
                self.score_box.insert(tk.END,
                    f"Neighbor {node} -> Score = {round(score,2)}\n")

            self.score_box.insert(tk.END,
                f"Selected Next Hop: {nxt}\n")

            path.append(nxt)
            visited.add(nxt)
            current = nxt

            self.draw(path)
            self.root.update()
            time.sleep(0.8)

        if current == self.dst:
            self.score_box.insert(tk.END,
                "\nDestination reached successfully.\n")
        else:
            self.score_box.insert(tk.END,
                "\nRouting stopped before reaching destination.\n")

# =====================================================
# MAIN FLOW
# =====================================================
root = tk.Tk()
root.withdraw()

algo = tk.StringVar(value=ALGO_CUSTOM)

sel = tk.Toplevel(root)
sel.title("Select Algorithm")

for a in [ALGO_GREEDY_NEIGHBOR, ALGO_CUSTOM]:
    tk.Radiobutton(sel,text=a,variable=algo,value=a).pack(anchor="w")

tk.Button(sel,text="Next",command=sel.destroy).pack()

root.wait_window(sel)

nodes=list(range(5))
edges=[(i,j) for i in nodes for j in nodes if i<j]

params=ParamInput(root,nodes,edges)
root.wait_window(params.top)

sim=tk.Toplevel(root)
Simulator(sim,algo.get(),params)

root.mainloop()
