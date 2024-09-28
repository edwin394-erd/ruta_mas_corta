import osmnx as ox
import os
import pickle
import folium
from sklearn.neighbors import NearestNeighbors
import heapq
import webbrowser
import tkinter as tk
from tkinter import simpledialog
from tkintermapview import TkinterMapView
import matplotlib.pyplot as plt



def cargar_grafo(place_name, directorio_grafos="grafos"):
    filename = place_name.replace(" ", "_") + ".pickle"
    filepath = os.path.join(directorio_grafos, filename)
    try:
        with open(filepath, 'rb') as f:
            G = pickle.load(f)
            print(f"Grafo de {place_name} cargado desde el archivo")
            return G
    except FileNotFoundError:
        print(f"Archivo del grafo de {place_name} no encontrado. Construyendo el grafo...")
        return None


def dijkstra(orig, dest, plot=False):
    for node in G.nodes:
        G.nodes[node]["visited"] = False
        G.nodes[node]["distance"] = float("inf")
        G.nodes[node]["previous"] = None
        G.nodes[node]["size"] = 0
    G.nodes[orig]["distance"] = 0
    G.nodes[orig]["size"] = 50
    G.nodes[dest]["size"] = 50
    pq = [(0, orig)]
    step = 0
    while pq:
        _, node = heapq.heappop(pq)
        if node == dest:
            if plot:
                print("Iteraciones:", step)
            return
        if G.nodes[node]["visited"]: continue
        G.nodes[node]["visited"] = True
        for edge in G.out_edges(node):
            neighbor = edge[1]
            weight = G.edges[(edge[0], edge[1], 0)]["weight"]
            if G.nodes[neighbor]["distance"] > G.nodes[node]["distance"] + weight:
                G.nodes[neighbor]["distance"] = G.nodes[node]["distance"] + weight
                G.nodes[neighbor]["previous"] = node
                heapq.heappush(pq, (G.nodes[neighbor]["distance"], neighbor))        
        step += 1


def distance(node1, node2):
    x1, y1 = G.nodes[node1]["x"], G.nodes[node1]["y"]
    x2, y2 = G.nodes[node2]["x"], G.nodes[node2]["y"]
    return ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    

def reconstruct_path(orig, dest, plot=False, algorithm=None):
    dist = 0
    speeds = []
    curr = dest
    coords_camino = []
    while curr != orig:
        prev = G.nodes[curr]["previous"]
        dist += G.edges[(prev, curr, 0)]["length"]
        speeds.append(G.edges[(prev, curr, 0)]["maxspeed"])
        coords_camino.append((G.nodes[curr]['y'], G.nodes[curr]['x']))  # Agrega las coordenadas a la lista

        if algorithm:
            G.edges[(prev, curr, 0)][f"{algorithm}_uses"] = G.edges[(prev, curr, 0)].get(f"{algorithm}_uses", 0) + 1
        curr = prev
    dist /= 1000

    if plot:
        # Crear el mapa centrado en el punto de inicio
        m = folium.Map(location=coords_camino[-1], zoom_start=15)
        # Agregar una polilínea (línea) al mapa
        folium.PolyLine(locations=coords_camino, color='blue', weight=2.5).add_to(m)
        # Agregar marcador en el punto de inicio
        folium.Marker(
            location=coords_camino[0],
            popup="FIn",
            icon=folium.Icon(color='red')
        ).add_to(m)
        # Agregar marcador en el punto final
        folium.Marker(
            location=coords_camino[-1],  # Último elemento de la lista de coordenadas
            popup="Inicio",
            icon=folium.Icon(color='green')
        ).add_to(m)
        # Guardar el mapa en un archivo HTML
        m.save('laruta.html')
        webbrowser.open('laruta.html')

        print(f"Distance: {dist}")
        print(f"Avg. speed: {sum(speeds)/len(speeds)}")
        print(f"Total time: {dist/(sum(speeds)/len(speeds)) * 60}")


place_name="zulia, venezuela"
directory_path = "grafos"
# Create "grafos" directory if it doesn't exist
os.makedirs(directory_path, exist_ok=True)
G= cargar_grafo(place_name)

if G is None:
    G = ox.graph_from_place(place_name, network_type="all")
    # Guardar el grafo en un archivo pickle
    with open(os.path.join("grafos", f"{place_name.replace(' ', '_')}.pickle"), 'wb') as f:
        pickle.dump(G, f)

for edge in G.edges:
    # Cleaning the "maxspeed" attribute, some values are lists, some are strings, some are None
    maxspeed = 40
    if "maxspeed" in G.edges[edge]:
        maxspeed = G.edges[edge]["maxspeed"]
        if type(maxspeed) == list:
            speeds = [ int(speed) for speed in maxspeed ]
            maxspeed = min(speeds)
        elif type(maxspeed) == str:
            maxspeed = int(maxspeed)
    G.edges[edge]["maxspeed"] = maxspeed
    # Adding the "weight" attribute (time = distance / speed)
    G.edges[edge]["weight"] = G.edges[edge]["length"] / maxspeed



# Crear la ventana principal de Tkinter
root = tk.Tk()
root.title("Mapa con TkinterMapView")
root.geometry("800x600")

# Preguntar la ciudad al usuario
#ciudad = simpledialog.askstring("Ciudad", "¿Qué ciudad deseas ver?")
ciudad= "maracaibo"

# Crear el widget del mapa
map_widget = TkinterMapView(root, width=800, height=600, corner_radius=0)

# Crear el botón "Buscar Ruta"
def buscar_ruta():
    print("Botón 'Buscar Ruta' presionado")
    root.destroy()
    print(point1)
    print(point2)
    # Get user-specified coordinates


    # Split the string into separate latitude and longitude values
    point1_str = f"({point1[0]}, {point1[1]})"
    start_coords_list = point1_str.split(",")

    if start_coords_list[0].startswith('('):
        # Remove the opening parenthesis and leading/trailing whitespace
        start_coords_list[0] = start_coords_list[0].strip('(')

    if start_coords_list[1].endswith(')'):
        # Remove the opening parenthesis and leading/trailing whitespace
        start_coords_list[1] = start_coords_list[1].strip(')')

    # Convert each value to a float
    start_coords = [float(coord.strip()) for coord in start_coords_list]


    # Split the string into separate latitude and longitude values
    point2_str = f"({point2[0]}, {point2[1]})"
    end_coords_list = point2_str.split(",")

    if end_coords_list[0].startswith('('):
        # Remove the opening parenthesis and leading/trailing whitespace
        end_coords_list[0] = end_coords_list[0].strip('(')

    if end_coords_list[1].endswith(')'):
        # Remove the opening parenthesis and leading/trailing whitespace
        end_coords_list[1] = end_coords_list[1].strip(')')
    
    # Convert each value to a float
    end_coords = [float(coord.strip()) for coord in end_coords_list]

    # Find nearest nodes to the specified coordinates
    start = ox.nearest_nodes(G, start_coords[1], start_coords[0])
    end = ox.nearest_nodes(G, end_coords[1], end_coords[0])

    dijkstra(start, end, plot=False)
    reconstruct_path(start, end, plot=True)



boton_buscar_ruta = tk.Button(root, text="Buscar Ruta", command=buscar_ruta)

# Usar grid para colocar los widgets
boton_buscar_ruta.grid(row=0, column=0, padx=10, pady=10)
map_widget.grid(row=1, column=0, sticky="nsew")

# Configurar la ventana para que el mapa se expanda
root.grid_rowconfigure(1, weight=1)
root.grid_columnconfigure(0, weight=1)

# Centrar el mapa en la ciudad ingresada
if ciudad:
    map_widget.set_address(ciudad)

# Variables para almacenar las coordenadas de los puntos
point1 = None
point2 = None

# Función para manejar el clic en el mapa
def on_map_click(event):
    global point1, point2
    if not point1:
        point1 = (event[0], event[1])
        map_widget.set_marker(event[0], event[1], text="Punto de Inicio")
        print("Coordenadas del Punto 1:", point1)
    elif not point2:
        point2 = (event[0], event[1])
        map_widget.set_marker(event[0], event[1], text="Punto de Fin")
        print("Coordenadas del Punto 2:", point2)



# Añadir evento de clic al mapa
map_widget.add_left_click_map_command(on_map_click)

# Iniciar el bucle principal de Tkinter
root.mainloop()
