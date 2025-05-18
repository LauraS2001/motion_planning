import osmnx as ox # library to interact with google maps
import random
import heapq # library to deal with heap structure

# Implement A* and compare the number of steps performed by A* vs the number of steps performed by Dijkstra using the same start and destination.
# Do this example with different cities in order to demonstrate that the bigger is the city the bigger is the distance from stating point to
# destination the bigger is the number of steps to perform Dijkstra respect to A*

def style_unvisited_edge(edge):        
    G.edges[edge]["color"] = "#d36206"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 0.2

def style_visited_edge(edge):
    #G.edges[edge]["color"] = "#d36206"
    G.edges[edge]["color"] = "green"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 1

def style_active_edge(edge):
    #G.edges[edge]["color"] = '#e8a900'
    G.edges[edge]["color"] = "red"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 1

def style_path_edge(edge):
    G.edges[edge]["color"] = "white"
    G.edges[edge]["alpha"] = 1
    G.edges[edge]["linewidth"] = 1

def plot_graph():
    ox.plot_graph(
        G,
        node_size =  [ G.nodes[node]["size"] for node in G.nodes ],
        node_color=[G.nodes[node].get("color", "white") for node in G.nodes],
        edge_color = [ G.edges[edge]["color"] for edge in G.edges ],
        edge_alpha = [ G.edges[edge]["alpha"] for edge in G.edges ],
        edge_linewidth = [ G.edges[edge]["linewidth"] for edge in G.edges ],
        #node_color = "white",
        bgcolor = "#18080e"
    )

def dijkstra(orig, dest, plot=False):
    for node in G.nodes:
        G.nodes[node]["visited"] = False
        G.nodes[node]["distance"] = float("inf")
        G.nodes[node]["previous"] = None
        G.nodes[node]["size"] = 0
    for edge in G.edges:
        style_unvisited_edge(edge)
    # Changing the style of the starting and destination node
    G.nodes[orig]["distance"] = 0
    G.nodes[orig]["size"] = 50
    G.nodes[dest]["size"] = 50
    G.nodes[dest]["color"] = "yellow"
    pq = [(0, orig)] # We start from origin
    step = 0
    while pq:
        _, node = heapq.heappop(pq) # heappop goes back the tuple with the lowest cost
        if node == dest: # We exit if the node is the destination
            print("Iterations:", step)
            plot_graph()
            return
        if G.nodes[node]["visited"]: continue
        G.nodes[node]["visited"] = True # if the node is not visited we mark it as visited
        for edge in G.out_edges(node): # This is the operation to take the node and possibly update the cost of that node
            style_visited_edge((edge[0], edge[1], 0))
            neighbor = edge[1]
            weight = G.edges[(edge[0], edge[1], 0)]["weight"]
            if G.nodes[neighbor]["distance"] > G.nodes[node]["distance"] + weight: # If the distance to the neighbor is higher than the distance to the neighbor passing from the node we are analysing
                G.nodes[neighbor]["distance"] = G.nodes[node]["distance"] + weight # we update the cost
                G.nodes[neighbor]["previous"] = node # We update the previous
                heapq.heappush(pq, (G.nodes[neighbor]["distance"], neighbor)) # We update the heap by adding this information
                for edge2 in G.out_edges(neighbor):
                    style_active_edge((edge2[0], edge2[1], 0))
        step += 1

# Function to reconstruct the best path starting from destination to starting point using previous node chain
def reconstruct_path(orig, dest, plot=False, algorithm=None):
    for edge in G.edges:
        style_unvisited_edge(edge)
    total_weight = 0
    dist = 0
    speeds = []
    curr = dest
    while curr != orig:
        prev = G.nodes[curr]["previous"]
        dist += G.edges[(prev, curr, 0)]["length"]
        speeds.append(G.edges[(prev, curr, 0)]["maxspeed"])
        total_weight = G.edges[(prev, curr, 0)]["weight"] + total_weight
        style_path_edge((prev, curr, 0))
        if algorithm:
            G.edges[(prev, curr, 0)][f"{algorithm}_uses"] = G.edges[(prev, curr, 0)].get(f"{algorithm}_uses", 0) + 1
        curr = prev
    dist /= 1000
    print("Total weight:", total_weight)

def plot_heatmap(algorithm):
    edge_colors = ox.plot.get_edge_colors_by_attr(G, f"{algorithm}_uses", cmap="hot")
    fig, _ = ox.plot_graph(
        G,
        node_size =  [ G.nodes[node]["size"] for node in G.nodes ],
        node_color=[G.nodes[node].get("color", "white") for node in G.nodes],
        # edge_color = edge_colors,
        edge_color = [ G.edges[edge]["color"] for edge in G.edges ],
        edge_alpha = [ G.edges[edge]["alpha"] for edge in G.edges ],
        edge_linewidth = [ G.edges[edge]["linewidth"] for edge in G.edges ],
        bgcolor = "#18080e"
    )

place_name = "Piedmont, California, USA"
#place_name = "Turin, Piedmont, Italy" # Selecting starting place
G = ox.graph_from_place(place_name, network_type="drive") # Building graph 'g' with osmnx with all roads where we can drive

for edge in G.edges:
    # Cleaning the "maxspeed" attribute, some values are lists, some are strings, some are None
    maxspeed = 40
    if "maxspeed" in G.edges[edge]:
        maxspeed = G.edges[edge]["maxspeed"]
        if type(maxspeed) == list:
            speeds = [ int(speed) for speed in maxspeed ]
            maxspeed = min(speeds)
        elif type(maxspeed) == str:
            maxspeed = maxspeed.strip(" mph")
            maxspeed = int(maxspeed)
    G.edges[edge]["maxspeed"] = maxspeed
    # Adding the "weight" attribute (time = distance / speed)
    G.edges[edge]["weight"] = (G.edges[edge]["length"] / 1000) / maxspeed * 3600


for edge in G.edges:
    G.edges[edge]["dijkstra_uses"] = 0

# I select 2 nodes from the graph and apply Dijkstra to them
nodelist = [(57, 10), (570, 10), (100, 1), (32, 200), (34, 80), (700, 397), (5, 577), (23, 2), (57, 269), (100, 67)]

for source, target in nodelist:
    start = list(G.nodes)[len(G.nodes) - source]
    end = list(G.nodes)[len(G.nodes) - target]
    print(f"start: {start}, end: {end}")

    print("Running Dijkstra")
    dijkstra(start, end)
    print("Done")

    reconstruct_path(start, end, algorithm="dijkstra", plot=True)  # Reconstructing the best path with Dijkstra
    plot_heatmap("dijkstra")

