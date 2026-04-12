import tkinter as tk
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import math
import time

# =====================================================
# LoRa PHY
# =====================================================
SF_SENS = {7:-123, 8:-126, 9:-129, 10:-132, 11:-134, 12:-137}

def compute_rssi(d, Pt=14, freq=433e6, n=3.0):
    if d < 1:
        d = 1
    PL0 = 32.44 + 20 * math.log10(freq/1e6)
    PL = PL0 + 10 * n * math.log10(d)
    return Pt - PL

# =====================================================
# PARAMETER WINDOW
# =====================================================
class ParamInput:
    def __init__(self, root, nodes, edges):
        self.top = tk.Toplevel(root)
        self.top.title("Emergency Mode Parameters")

        # Spreading Factor
        self.sf = tk.IntVar(value=9)
        tk.Label(self.top, text="Spreading Factor").grid(row=0,column=0)
        tk.OptionMenu(self.top, self.sf, *SF_SENS.keys()).grid(row=0,column=1)

        # RSSI Threshold
        tk.Label(self.top, text="RSSI Threshold (dBm)").grid(row=1,column=0)
        self.rssi_th = tk.Entry(self.top,width=6)
        self.rssi_th.insert(0,"-120")
        self.rssi_th.grid(row=1,column=1)

        # Battery Input
        self.battery={}
        tk.Label(self.top,text="Battery (%)").grid(row=2,column=0)
        for i,n in enumerate(nodes):
            e=tk.Entry(self.top,width=5)
            e.insert(0,"80")
            e.grid(row=3+i,column=1)
            tk.Label(self.top,text=f"Node {n}").grid(row=3+i,column=0)
            self.battery[n]=e

        # Distance per link
        self.dist={}
        tk.Label(self.top,text="Distance (m)").grid(row=2,column=2)

        for i,(u,v) in enumerate(edges):
            e=tk.Entry(self.top,width=6)
            e.insert(0,"100")
            e.grid(row=3+i,column=3)
            tk.Label(self.top,text=f"{u}-{v}").grid(row=3+i,column=2)
            self.dist[(u,v)] = e

        # Source
        self.src=tk.IntVar(value=0)
        tk.Label(self.top,text="Source").grid(row=20,column=0)
        tk.OptionMenu(self.top,self.src,*nodes).grid(row=20,column=1)

        tk.Button(self.top,text="Start",command=self.submit)\
            .grid(row=21,column=0,columnspan=2)

    def submit(self):
        self.batt_vals={n:int(e.get()) for n,e in self.battery.items()}
        self.sf_val=self.sf.get()
        self.source=self.src.get()
        self.rssi_threshold=float(self.rssi_th.get())
        self.dist_vals={k:float(e.get()) for k,e in self.dist.items()}
        self.top.destroy()

# =====================================================
# SIMULATOR
# =====================================================
class Simulator:
    def __init__(self, root, params):
        self.root=root
        self.battery=params.batt_vals
        self.sf=params.sf_val
        self.src=params.source
        self.rssi_threshold=params.rssi_threshold

        self.root.title("LoRa Emergency Broadcast Simulation")

        # Fixed node layout
        self.pos={
            0:(0,1),1:(0.95,0.31),2:(0.59,-0.81),
            3:(-0.59,-0.81),4:(-0.95,0.31)
        }

        self.G=nx.Graph()
        self.G.add_nodes_from(self.pos.keys())

        self.rssi={}
        sens=SF_SENS[self.sf]

        # Build edges using RSSI from distance
        for (u,v),d in params.dist_vals.items():
            r=compute_rssi(d)
            if r>=sens:
                self.G.add_edge(u,v)
                self.rssi[(u,v)] = self.rssi[(v,u)] = r

        # Matplotlib
        self.fig,self.ax=plt.subplots(figsize=(5,5))
        self.canvas=FigureCanvasTkAgg(self.fig,root)
        self.canvas.get_tk_widget().pack()

        # Log box
        self.log=tk.Text(root,height=10,width=75)
        self.log.pack()

        tk.Button(root,text="Run Emergency Broadcast",
                  command=self.run).pack()

        self.draw()

    def draw(self,highlight=None):
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

        if highlight:
            nx.draw_networkx_edges(self.G,self.pos,
                                   edgelist=highlight,
                                   edge_color="red",width=3)

        self.canvas.draw()

    def run(self):
        visited=set()
        queue=[self.src]
        highlight=[]

        self.log.delete("1.0",tk.END)
        self.log.insert(tk.END,"Emergency Broadcast Started\n\n")

        while queue:
            current=queue.pop(0)

            if current in visited:
                continue

            visited.add(current)
            self.log.insert(tk.END,f"Node {current} activated\n")

            for n in self.G.neighbors(current):

                if (self.rssi[(current,n)] >= self.rssi_threshold) and (n not in visited):

                    highlight.append((current,n))
                    queue.append(n)

                    self.log.insert(tk.END,
                        f"Broadcast {current} -> {n} "
                        f"({int(self.rssi[(current,n)])} dBm)\n")

            self.draw(highlight)
            self.root.update()
            time.sleep(0.8)

        self.log.insert(tk.END,"\nEmergency Broadcast Complete\n")

# =====================================================
# MAIN
# =====================================================
root=tk.Tk()
root.withdraw()

nodes=list(range(5))
edges=[(i,j) for i in nodes for j in nodes if i<j]

params=ParamInput(root,nodes,edges)
root.wait_window(params.top)

sim=tk.Toplevel(root)
Simulator(sim,params)

root.mainloop()
