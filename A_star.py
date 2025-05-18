import math
from collections import Counter
import osmnx as ox # library to interact with google maps
from geopy.distance import great_circle
import heapq # library to deal with heap structure

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

def A_star(orig, dest, plot=False):
    for node in G.nodes:
        G.nodes[node]["visited"] = False
        G.nodes[node]["distance"] = float("inf")
        G.nodes[node]["previous"] = None
        G.nodes[node]["size"] = 0
        G.nodes[node]["gScore"] = float("inf") # Path cost from the start node to the node n
        G.nodes[node]["fScore"] = float("inf") # Global cost value
    G.nodes[orig]["gScore"] = 0
    G.nodes[orig]["hScore"] = calculation_haversine_heuristic(orig, dest)
    G.nodes[orig]["fScore"] = G.nodes[orig]["gScore"] + G.nodes[orig]["hScore"]

    for edge in G.edges:
        style_unvisited_edge(edge)
    # Changing the style of the starting and destination node
    G.nodes[orig]["distance"] = 0
    G.nodes[orig]["size"] = 50
    G.nodes[dest]["size"] = 50
    G.nodes[dest]["color"] = "cyan"
    # Put in pq the tuple of fScore and node
    pq = [(0, orig)] # We start from origin
    pq_nodes = {orig} # Set of points in pq
    step = 0
    while pq:
        _, current = heapq.heappop(pq) # heappop goes back the tuple with the lowest fScore
        pq_nodes.remove(current)
        if current == dest: # We exit if the node is the destination
            print("Iterations:", step)
            plot_graph()
            return
        if G.nodes[current]["visited"]: continue
        G.nodes[current]["visited"] = True # if the node is not visited we mark it as visited
        for edge in G.out_edges(current): # This take the tuple (node, neighbor) in this way we take all neighbor of the current node
            style_visited_edge((edge[0], edge[1], 0))
            neighbor = edge[1]
            weight = G.edges[(edge[0], edge[1], 0)]["weight"] # This take the cost of the connection between current and neighbor
            tentative_gScore = G.nodes[current]["gScore"] + weight
            if tentative_gScore < G.nodes[neighbor]["gScore"]:
                G.nodes[neighbor]["hScore"] = calculation_haversine_heuristic(neighbor, dest)  # Heuristic: cost from the n node to the destination
                G.nodes[neighbor]["distance"] = G.nodes[current]["distance"] + weight # we update the cost
                G.nodes[neighbor]["gScore"] = tentative_gScore
                G.nodes[neighbor]["fScore"]  = tentative_gScore + G.nodes[neighbor]["hScore"]
                G.nodes[neighbor]["previous"] = current  # We update the previous
                if neighbor not in pq_nodes:
                    heapq.heappush(pq, (G.nodes[neighbor]["fScore"], neighbor)) # We update the heap by adding the neighbor and its fscore
                    pq_nodes.add(neighbor)
                for edge2 in G.out_edges(neighbor):
                    style_active_edge((edge2[0], edge2[1], 0))

        step += 1

def calculation_Manhattan_geometry_heuristic(node, dest):
    dx = abs(G.nodes[node]['x'] - G.nodes[dest]['x'])
    dy = abs(G.nodes[node]['y'] - G.nodes[dest]['y'])
    # Conversion to km
    dx_km = dx * 111.32 * math.cos(math.radians(G.nodes[node]['y']))
    dy_km = dy * 110.57

    distance_km = math.sqrt(dx_km ** 2 + dy_km ** 2)
    D = 1
    return D * distance_km / average_speed * 3600

def calculation_euclidean_distance_heuristic(node, dest):
    dx = abs(G.nodes[node]['x'] - G.nodes[dest]['x'])
    dy = abs(G.nodes[node]['y'] - G.nodes[dest]['y'])
    # Conversion to km
    dx_km = dx * 111.32 * math.cos(math.radians(G.nodes[node]['y']))
    dy_km = dy * 110.57

    distance_km = math.sqrt(dx_km ** 2 + dy_km ** 2)
    D = 1
    return D * math.sqrt(dx_km * dx_km + dy_km * dy_km) / average_speed * 3600

def calculation_haversine_heuristic(node, dest):
    coord1 = (G.nodes[node]['y'], G.nodes[node]['x'])
    coord2 = (G.nodes[dest]['y'], G.nodes[dest]['x'])
    return great_circle(coord1, coord2).km / average_speed * 3600

# Function to reconstruct the best path starting from destination to starting point using previous node chain
def reconstruct_path(orig, dest, plot=False, algorithm=None):
    for edge in G.edges:
        style_unvisited_edge(edge)
    dist = 0
    total_weight = 0
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

#place_name = "Piedmont, California, USA"
place_name = "Turin, Piedmont, Italy" # Selecting starting place
G = ox.graph_from_place(place_name, network_type="drive") # Building graph 'g' with osmnx with all roads where we can drive

average_speed = 0
count = 0
max_speed = 0

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
    #G.edges[edge]["weight"] = G.edges[edge]["length"] / maxspeed # Associating weight to each edge
    G.edges[edge]["weight"] = (G.edges[edge]["length"] / 1000) / maxspeed * 3600 # Associating weight to each edge
    if maxspeed > max_speed:
        max_speed = maxspeed
    average_speed += maxspeed
    count += 1

average_speed = average_speed / count
print("Average speed:", average_speed)
print("Max speed:", max_speed)


for edge in G.edges:
    G.edges[edge]["A_star_uses"] = 0

# List of offsets to select start points and destinations of each route
nodelist = [(57, 10), (570, 10), (100, 1), (32, 200), (34, 80), (700, 397), (5, 577), (23, 2), (57, 269), (100, 67)]

for source, target in nodelist:
    start = list(G.nodes)[len(G.nodes) - source]
    end = list(G.nodes)[len(G.nodes) - target]
    print(f"start: {start}, end: {end}")

    print("Running A*")
    A_star(start, end)
    print("Done")

    reconstruct_path(start, end, algorithm="A_star", plot=True)  # Reconstructing the best path
    plot_heatmap("A_star")

