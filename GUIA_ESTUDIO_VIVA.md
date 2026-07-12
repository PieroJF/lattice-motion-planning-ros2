---
title: "Guia de Estudio Completa - VIVA Assignment 2"
subtitle: "Path Planning & Trajectory Generation - RMPC"
geometry: margin=2cm
fontsize: 11pt
toc: true
toc-depth: 3
header-includes:
  - \usepackage{fancyhdr}
  - \pagestyle{fancy}
  - \fancyhead[L]{RMPC Assignment 2 - Guia de Viva}
  - \fancyhead[R]{\thepage}
  - \usepackage{tcolorbox}
  - \newtcolorbox{questionbox}{colback=blue!5!white, colframe=blue!75!black, title=Pregunta Probable}
  - \newtcolorbox{answerbox}{colback=green!5!white, colframe=green!50!black}
  - \usepackage{booktabs}
  - \usepackage{longtable}
---

\newpage

# 1. VISION GENERAL DEL PROYECTO

El proyecto implementa **3 algoritmos de planificacion de caminos** y un **generador de trayectorias** para un robot movil en un entorno con obstaculos.

## Arquitectura de Archivos

```
main.py                          -- Orquestador: crea mapa, ejecuta planners, grafica
path_planner/
  utils.py                       -- Estructura Graph + ObstaclesGrid (collision checker)
  lattice_planner.py             -- Lattice graph + A*
  rrt_planner.py                 -- Rapidly-exploring Random Tree
  prm_planner.py                 -- Probabilistic Roadmap
trajectory_generator/
  traj_generation.py             -- Resampling, perfil de velocidad, interpolacion
```

## Flujo de Datos Completo (main.py)

1. Crear grid de lattice 10x10 (= 400 vertices con 4 orientaciones)
2. Definir obstaculos en mapa booleano 100x100
3. Invalidar aristas que cruzan obstaculos (peso = inf)
4. A\* encuentra camino optimo en el lattice
5. RRT encuentra camino en espacio continuo
6. PRM construye roadmap y busca camino
7. Interpolar el path del lattice (densificar)
8. Generar trayectoria con perfil de velocidad trapezoidal
9. Visualizar resultados

## El Mapa de Obstaculos

![Mapa de obstaculos con start (1,8,90) y goal (8,2,270)](RMPC_Assignment2_ENTREGA/report_figures/fig1_obstacle_map.png){ width=65% }

**Codigo que lo define** (`main.py` lineas 37-48):

```python
obs.map[25:35, 45:56] = True      # bloque superior-centro
obs.map[40:43, 48:76] = True      # barra horizontal
obs.map[67:89, 57:76] = True      # bloque inferior-derecha
obs.map[50:55, 60:89] = True      # barra inferior
obs.map[20:60, 5:35] = True       # bloque grande izquierda
```

- **Start**: `(1, 8, 90)` -- fila 1, columna 8, mirando "arriba" (90 grados)
- **Goal**: `(8, 2, 270)` -- fila 8, columna 2, mirando "abajo" (270 grados)

## Conceptos Clave del Lattice

- **Vertice del lattice**: tupla `(row, col, angle)`. Row y col son la posicion en el grid, angle es la orientacion del robot (0, 90, 180 o 270 grados).
- **Aristas rectas**: peso = 1 (una celda de distancia).
- **Aristas arco**: peso = pi (cuarto de circulo).
- **Obstaculos**: array booleano numpy 100x100. Una arista es invalida si cualquier punto muestreado a lo largo de ella cae en un obstaculo.

\newpage

# 2. ALGORITMO A\* SOBRE LATTICE

**Archivo**: `lattice_planner.py`

## 2.1 Estructura del Grafo Lattice

Cada celda del grid 10x10 tiene 4 orientaciones posibles = **400 vertices totales**.

**Aristas rectas** (peso = 1): El robot avanza una celda en la direccion de su heading.

- Angulo 0 $\rightarrow$ se mueve a la derecha (col+1)
- Angulo 90 $\rightarrow$ se mueve hacia arriba (row-1)
- Angulo 180 $\rightarrow$ se mueve a la izquierda (col-1)
- Angulo 270 $\rightarrow$ se mueve hacia abajo (row+1)

**Aristas curvas/arco** (peso = $\pi$): El robot gira 90 grados mientras avanza a una celda diagonal.

**Codigo de generacion** (`lattice_planner.py` lineas 66-183):

```python
# Vertices: 4 angulos por cada celda (linea 75-79)
for row in range(n_rows):
    for col in range(n_cols):
        for angle in [0, 90, 180, 270]:
            v = (row, col, angle)
            self._graph.add_vertex(v)

# Arista recta - moverse hacia abajo (linea 101-103)
if (row + 1) < n_rows and angle == 270:
    v_buttom = (row + 1, col, 270)
    self._graph.set_edge(v, v_buttom, 1)          # peso = 1

# Arista curva - giro diagonal (linea 105-107)
if (col - 1) >= 0 and (row + 1) < n_rows and angle == 270:
    v_buttom_left = (row + 1, col - 1, 180)
    self._graph.set_edge(v, v_buttom_left, np.pi)  # peso = pi
```

\begin{questionbox}
"Por que el peso de un arco es pi y no sqrt(2)?"
\end{questionbox}
\begin{answerbox}
El arco es un cuarto de circulo de radio = 1 celda. Su longitud es $(2\pi r)/4 = \pi r/2$. Con el radio normalizado, el codigo usa $\pi$ como aproximacion del costo de recorrer esa curva. Es mayor que una linea recta (1) porque la curva es mas larga.
\end{answerbox}

## 2.2 Invalidacion de Obstaculos

`update_obstacles()` (`lattice_planner.py` lineas 34-47):

```python
def update_obstacles(self, obs):
    for edge_key, edge_val in self._graph._edge_dict.items():
        is_valid = obs.is_edge_valid(edge_key, edge_val,
                        self.lattice_cell_size, self.arc_primitives)
        if not is_valid:
            self._graph._edge_dict[edge_key] = np.inf  # arista bloqueada
    self._graph.set_adjacency_matrix()
```

Se muestrea cada arista (linea recta o arco) y si **cualquier punto** cae dentro de un obstaculo, el peso se pone en `inf`. A\* nunca elegira esa arista.

\newpage

## 2.3 Implementacion de A\*

**Lineas 209-238** de `lattice_planner.py`

![Camino encontrado por A\* sobre el lattice](RMPC_Assignment2_ENTREGA/report_figures/fig2_astar_path.png){ width=65% }

```python
distances[s] = 0                    # g(start) = 0
costs[s] = self.calH(s, g)          # f(start) = 0 + h(start)

while not open_set.empty():
    cost, curr = open_set.get()     # pop nodo con menor f

    if curr in closed_set:          # lazy deletion: saltar duplicados
        continue
    closed_set.add(curr)            # marcar como expandido

    if curr == g:                   # goal check DESPUES de popping
        path = self.traverse_path(s, g, parent_node)
        return path

    neighbors = self.get_neighbor(curr, graph_vert_list, adjacency_matrix)
    for n in neighbors:
        if n in closed_set:
            continue

        g_cost = distances[curr] + self.cal_expand_cost(curr, n, edge_dict)

        if n not in distances or g_cost < distances[n]:
            distances[n] = g_cost
            f = g_cost + self.calH(n, g)     # f = g + h
            costs[n] = f
            parent_node[n] = curr
            open_set.put((f, n))             # push con prioridad f

print("no path found")
return None
```

### Tabla de conceptos clave

| Concepto | Que es | En tu codigo |
|----------|--------|-------------|
| g-cost | Costo real desde start hasta nodo | `distances[n]` |
| h-cost | Estimacion desde nodo hasta goal | `calH(n, g)` |
| f-cost | g + h, costo total estimado | `f = g_cost + self.calH(n, g)` |
| Open set | Nodos descubiertos no expandidos | `PriorityQueue` (min-heap) |
| Closed set | Nodos ya expandidos | `set()` |
| Lazy deletion | Push duplicados, skip al pop | `if curr in closed_set: continue` |

\begin{questionbox}
"Por que revisas el goal DESPUES de sacarlo de la cola y no cuando lo agregas?"
\end{questionbox}
\begin{answerbox}
Si lo reviso al agregar, podria encontrar el goal por un camino suboptimo primero. Al revisarlo despues de popping, el PriorityQueue garantiza que la primera vez que saco el goal, su f-cost es el minimo. Con una heuristica admisible, esto garantiza optimalidad.
\end{answerbox}

\begin{questionbox}
"Que es lazy deletion y por que la usas?"
\end{questionbox}
\begin{answerbox}
El PriorityQueue de Python no tiene operacion \texttt{decrease-key}. Cuando encuentro un mejor camino a un nodo, no puedo actualizar su prioridad en la cola. En su lugar, push una nueva entrada con la nueva prioridad. Cuando pop una entrada vieja (el nodo ya esta en closed\_set), simplemente la ignoro. Es correcto y O(E log V) en practica.
\end{answerbox}

## 2.4 Reconstruccion del Camino

**Lineas 255-262** de `lattice_planner.py`:

```python
path = []
curr = g
while curr != s:
    path.append(curr)
    curr = parent_node[curr]
path.append(s)
path.reverse()
return path
```

Backtracking clasico: desde goal, sigo los parent pointers hasta start, luego invierto.

\begin{questionbox}
"Por que no construyes el camino hacia adelante desde start?"
\end{questionbox}
\begin{answerbox}
Durante la busqueda, solo registro el padre de cada nodo (\texttt{parent\_node[child] = parent}), no sus hijos. Solo puedo trazar de hijo a padre, por eso voy de goal a start y luego invierto.
\end{answerbox}

## 2.5 Heuristica: Distancia Euclidiana

**Lineas 307-309** de `lattice_planner.py`:

```python
dx = v1[0] - v2[0]
dy = v1[1] - v2[1]
return np.sqrt(dx**2 + dy**2)
```

Distancia euclidiana entre (row, col) del nodo actual y el goal. Ignora el angulo.

\begin{questionbox}
"Es tu heuristica admisible? Demuestralo."
\end{questionbox}
\begin{answerbox}
Si. La distancia euclidiana es la linea recta entre dos puntos, que es siempre $\leq$ cualquier camino real (que debe recorrer aristas de peso 1 o $\pi$). Nunca sobreestima, por lo tanto es admisible.
\end{answerbox}

\begin{questionbox}
"Es consistente (monotona)?"
\end{questionbox}
\begin{answerbox}
Si. Para cualquier nodo n y vecino n': $h(n) \leq cost(n,n') + h(n')$. Esto se cumple por la desigualdad triangular de la distancia euclidiana. Consistencia implica admisibilidad, y garantiza que A* no necesita reabrir nodos.
\end{answerbox}

\begin{questionbox}
"Por que ignoras el angulo en la heuristica?"
\end{questionbox}
\begin{answerbox}
La heuristica solo necesita ser una cota inferior. Incluir diferencias de angulo la haria mas informada pero tambien mas compleja. La euclidiana en (row,col) ya es admisible y funciona bien.
\end{answerbox}

\begin{questionbox}
"Podrias usar Manhattan como heuristica?"
\end{questionbox}
\begin{answerbox}
Si, pero seria menos informada que euclidiana porque el lattice permite movimientos diagonales via arcos. Manhattan sobreestimaria para caminos diagonales. Euclidiana es mas tight (mas cercana al costo real).
\end{answerbox}

## 2.6 Costo de Expansion

**Linea 291** de `lattice_planner.py`:

```python
return edge_dict[(v1, v2)]
```

Busca el peso de la arista en el diccionario. Rectas = 1, arcos = $\pi$, bloqueadas = $\infty$.

\newpage

# 3. RRT - Rapidly-exploring Random Tree

**Archivo**: `rrt_planner.py`

**Parametros**: `max_iter=500`, `step_size=5`, mapa 100x100

![Arbol RRT y camino encontrado](RMPC_Assignment2_ENTREGA/report_figures/fig3_rrt_path.png){ width=55% }

## 3.1 Algoritmo Principal

**Lineas 40-59** de `rrt_planner.py`:

```python
def plan(self):
    for i in range(self.max_iter):
        rand_node = self.sample_random_point()         # 1. Muestra aleatoria
        nearest_node = self.find_nearest_node(rand_node)  # 2. Nodo mas cercano
        new_node = self.steer(nearest_node, rand_node)    # 3. Avanzar

        if new_node and not self.is_colliding(new_node, nearest_node):
            self.tree.append(new_node)                    # 4. Agregar al arbol
            if self.reached_goal(new_node):               # 5. Llegamos?
                return self.construct_path(new_node)

    print("Path not found.")
    return None
```

## 3.2 Muestreo Aleatorio

**Lineas 68-70**:

```python
rx = np.random.randint(0, self.map_size[0])
ry = np.random.randint(0, self.map_size[1])
return Node(rx, ry)
```

Muestreo uniforme en el mapa. Usa enteros porque el mapa de obstaculos es un grid discreto.

\begin{questionbox}
"Por que no agregas goal biasing?"
\end{questionbox}
\begin{answerbox}
Goal biasing (muestrear el goal un 5-10\% del tiempo) aceleraria la convergencia. Lo mantuve simple porque con 500 iteraciones y step\_size=5 en un mapa 100x100, converge bien. No era requerido por el assignment.
\end{answerbox}

## 3.3 Busqueda del Vecino Mas Cercano

**Lineas 82-89**:

```python
best = None
best_d = float('inf')
for n in self.tree:
    d = np.sqrt((n.x - rand_node.x)**2 + (n.y - rand_node.y)**2)
    if d < best_d:
        best_d = d
        best = n
return best
```

Busqueda lineal O(n). Recorre todos los nodos del arbol.

\begin{questionbox}
"Esto es O(n). Como lo mejorarias?"
\end{questionbox}
\begin{answerbox}
Usando un \textbf{KD-Tree} (como \texttt{scipy.spatial.KDTree}) para queries O(log n). De hecho, en mi PRM planner si use KDTree. Para RRT con max 500 iteraciones, la busqueda lineal es suficiente.
\end{answerbox}

## 3.4 Steer - Avanzar Hacia la Muestra

**Lineas 102-109**:

```python
dx = rand_node.x - nearest_node.x
dy = rand_node.y - nearest_node.y
d = np.sqrt(dx**2 + dy**2)
if d <= self.step_size:
    return Node(rand_node.x, rand_node.y, nearest_node)
nx = nearest_node.x + dx / d * self.step_size   # vector unitario * step_size
ny = nearest_node.y + dy / d * self.step_size
return Node(nx, ny, nearest_node)
```

Si la muestra esta dentro de step\_size, la usa directamente. Si no, avanza exactamente step\_size unidades en esa direccion. `dx/d` y `dy/d` forman el vector unitario de direccion.

\begin{questionbox}
"Por que limitar el step size?"
\end{questionbox}
\begin{answerbox}
Sin limite, el arbol podria saltar sobre obstaculos. El step\_size asegura crecimiento incremental y que el collision check entre los dos nodos sea confiable sobre distancias cortas. Tambien controla la densidad del arbol.
\end{answerbox}

## 3.5 Deteccion de Colisiones

**Lineas 122-132**:

```python
dist = np.sqrt((new_node.x - nearest_node.x)**2 +
               (new_node.y - nearest_node.y)**2)
n_steps = max(int(dist), 1)
for i in range(n_steps + 1):
    t = i / n_steps
    px = int(nearest_node.x + t * (new_node.x - nearest_node.x))
    py = int(nearest_node.y + t * (new_node.y - nearest_node.y))
    if px < 0 or px >= self.map_size[0] or \
       py < 0 or py >= self.map_size[1]:
        return True
    if self.obstacles.map[px, py]:
        return True
return False
```

Muestrea puntos a lo largo del segmento de linea (~1 punto por unidad de distancia). Si cualquier punto esta en obstaculo o fuera del mapa, hay colision.

\begin{questionbox}
"Podria tu collision check fallar (miss un obstaculo)?"
\end{questionbox}
\begin{answerbox}
Con n\_steps = int(dist) muestreo $\sim$1 punto por unidad. Como los obstaculos ocupan celdas de 1x1, es generalmente suficiente. Para un check perfecto, usaria el algoritmo de Bresenham para enumerar todas las celdas que la linea atraviesa. Pero para step\_size=5, la densidad de muestreo es adecuada.
\end{answerbox}

## 3.6 Goal Check y Reconstruccion de Path

**reached\_goal** (lineas 144-145):

```python
d = np.sqrt((new_node.x - self.goal.x)**2 +
            (new_node.y - self.goal.y)**2)
return d <= self.step_size
```

**construct\_path** (lineas 157-163):

```python
path = []
node = end_node
while node is not None:        # root tiene parent = None
    path.append((node.x, node.y))
    node = node.parent
path.reverse()
return path
```

\begin{questionbox}
"Es RRT optimo?"
\end{questionbox}
\begin{answerbox}
\textbf{No.} RRT es \textbf{probabilisticamente completo} (si existe un camino, lo encontrara con suficientes iteraciones), pero NO garantiza el camino mas corto. Para optimalidad se necesita \textbf{RRT*}, que re-conecta el arbol para reducir costos.
\end{answerbox}

\begin{questionbox}
"Que es completitud probabilistica?"
\end{questionbox}
\begin{answerbox}
Conforme el numero de iteraciones tiende a infinito, la probabilidad de encontrar un camino (si existe) tiende a 1. No es determinista: una ejecucion individual puede fallar, pero con suficiente tiempo, tendra exito.
\end{answerbox}

\begin{questionbox}
"Como afecta step\_size al rendimiento de RRT?"
\end{questionbox}
\begin{answerbox}
Muy pequeno: expansion lenta, muchas iteraciones. Muy grande: podria saltar sobre pasajes estrechos, collision checks menos confiables. step\_size=5 en mapa 100x100 es un buen balance.
\end{answerbox}

\newpage

# 4. PRM - Probabilistic Roadmap

**Archivo**: `prm_planner.py`

**Parametros**: `num_samples=200`, `k_neighbors=10`, `step_size=5`

![Roadmap PRM con 202 nodos y 1114 aristas](RMPC_Assignment2_ENTREGA/report_figures/fig4_prm_path.png){ width=55% }

## Dos Fases: Construccion + Consulta

## 4.1 Fase 1: Construccion del Roadmap

**Lineas 49-69** de `prm_planner.py`:

```python
# Agregar start y goal primero
self.roadmap.append(self.start)
self.roadmap.append(self.goal)

# Muestrear puntos libres de obstaculos
for _ in range(self.num_samples):
    p = self.sample_free_point()
    if p is not None:
        self.roadmap.append(p)

# Conectar cada nodo a sus k vecinos mas cercanos
for node in self.roadmap:
    nearest = self.find_k_nearest(node, self.k_neighbors)
    for nb in nearest:
        if not self.is_colliding(node, nb):
            # Aristas bidireccionales (grafo no dirigido)
            if node not in self.edges:
                self.edges[node] = []
            if nb not in self.edges:
                self.edges[nb] = []
            if nb not in self.edges[node]:
                self.edges[node].append(nb)
            if node not in self.edges[nb]:
                self.edges[nb].append(node)
```

\begin{questionbox}
"Por que agregas start y goal primero?"
\end{questionbox}
\begin{answerbox}
Si no estan en el roadmap, no se puede encontrar un camino hacia/desde ellos. Deben participar en el proceso de conexion con vecinos.
\end{answerbox}

## 4.2 Muestreo Libre de Obstaculos

**Lineas 78-83**:

```python
for _ in range(100):    # hasta 100 intentos (rejection sampling)
    x = np.random.randint(0, self.map_size[0])
    y = np.random.randint(0, self.map_size[1])
    if not self.obstacles.map[x, y]:
        return Node(x, y)
return None
```

## 4.3 K Vecinos Mas Cercanos con KD-Tree

**Lineas 96-107**:

```python
pts = np.array([[n.x, n.y] for n in self.roadmap])
tree = KDTree(pts)
k_actual = min(k + 1, len(self.roadmap))  # k+1 porque incluye al propio nodo
_, idxs = tree.query([node.x, node.y], k=k_actual)

result = []
for i in idxs:
    nb = self.roadmap[i]
    if nb.x == node.x and nb.y == node.y:  # excluir el propio nodo
        continue
    result.append(nb)
return result[:k]
```

\begin{questionbox}
"Reconstruyes el KDTree en cada llamada. No es ineficiente?"
\end{questionbox}
\begin{answerbox}
Si, O(n log n) por llamada $\times$ n llamadas = O($n^2$ log n) total. Idealmente construiria el tree una sola vez. Con 200 nodos es negligible.
\end{answerbox}

\begin{questionbox}
"Por que KDTree en PRM pero busqueda lineal en RRT?"
\end{questionbox}
\begin{answerbox}
PRM conecta TODOS los nodos a sus k vecinos, llamando find\_k\_nearest 200+ veces. KDTree hace cada query O(log n) en vez de O(n). Para RRT, find\_nearest se llama max 500 veces en un arbol creciente, y la busqueda lineal simple era suficiente.
\end{answerbox}

## 4.4 Fase 2: Busqueda con Dijkstra

**Lineas 139-189** de `prm_planner.py`:

```python
from queue import PriorityQueue
dist = {}; prev = {}; visited = set()
pq = PriorityQueue()

sk = (self.start.x, self.start.y)   # tuplas como claves
gk = (self.goal.x, self.goal.y)
dist[sk] = 0
pq.put((0, sk))

lookup = {}    # tupla -> nodo
for n in self.roadmap:
    lookup[(n.x, n.y)] = n

while not pq.empty():
    c, curr = pq.get()
    if curr in visited: continue
    visited.add(curr)

    if curr == gk:          # encontramos el goal
        path = []; k = gk
        while k in prev:
            path.append(k); k = prev[k]
        path.append(sk); path.reverse()
        return path

    node = lookup.get(curr)
    if node is None or node not in self.edges: continue

    for nb in self.edges[node]:
        nbk = (nb.x, nb.y)
        if nbk in visited: continue
        w = np.sqrt((node.x - nb.x)**2 + (node.y - nb.y)**2)
        new_c = dist[curr] + w
        if nbk not in dist or new_c < dist[nbk]:
            dist[nbk] = new_c
            prev[nbk] = curr
            pq.put((new_c, nbk))

print("PRM: no path found")
return None
```

Es **Dijkstra** (A\* con h=0). Los pesos son las distancias euclidianas entre nodos conectados.

\begin{questionbox}
"Por que Dijkstra en lugar de A*?"
\end{questionbox}
\begin{answerbox}
Ambos funcionan. Dijkstra es mas simple (no necesita heuristica). Con $\sim$200 nodos, la diferencia de rendimiento es despreciable.
\end{answerbox}

\begin{questionbox}
"Por que tuplas como claves de diccionario en vez de objetos Node?"
\end{questionbox}
\begin{answerbox}
Los objetos Node no son hashables por contenido (se hashean por identidad). Dos objetos Node con las mismas coordenadas serian claves diferentes. Las tuplas (x,y) se comparan por valor, lo cual es correcto.
\end{answerbox}

\newpage

# 5. COMPARACION DE LOS 3 PLANIFICADORES

![Comparacion visual: A* Lattice, RRT y PRM](RMPC_Assignment2_ENTREGA/report_figures/fig9_comparison.png){ width=95% }

| Propiedad | A\* Lattice | RRT | PRM |
|-----------|-----------|-----|-----|
| Tipo de espacio | Discreto (lattice) | Continuo | Continuo |
| Optimalidad | Si (con h admisible) | No | No (aprox.) |
| Completitud | Completo | Prob. completo | Prob. completo |
| Multi-query | Si (mismo grafo) | No (arbol nuevo) | Si (roadmap reusable) |
| Orientacion | Si (4 headings) | No (solo x,y) | No (solo x,y) |

\begin{questionbox}
"Cual planificador es mejor?"
\end{questionbox}
\begin{answerbox}
Depende del contexto: \textbf{A*} cuando necesitas optimalidad y el robot tiene restricciones de orientacion. \textbf{RRT} para single-query en espacios de alta dimension. \textbf{PRM} cuando necesitas resolver muchas queries en el mismo entorno.
\end{answerbox}

\newpage

# 6. GENERACION DE TRAYECTORIA

**Archivo**: `traj_generation.py`

La trayectoria convierte un **path discreto** (waypoints) en un **perfil continuo** con posicion, velocidad, aceleracion y velocidad angular parametrizados por tiempo.

## 6.1 Interpolacion del Path

**Lineas 164-192** de `traj_generation.py`. Densifica el path del lattice:

```python
# Segmentos rectos (misma orientacion): lineas 182-186
if path[i][2] == path[i+1][2]:
    for j in range(lattice_cell_size):
        row = v1[0]*lattice_cell_size + j*dir_row/lattice_cell_size
        col = v1[1]*lattice_cell_size + j*dir_col/lattice_cell_size
        sampled_path.append((row/cell_size, col/cell_size, v1[2]))

# Arcos (diferente orientacion): lineas 187-191
else:
    arc = graph.arc_primitives[(v1[2], v2[2])]
    arc = np.array(v1[:2]).reshape((2,1)) * lattice_cell_size + arc
    for j in range(arc.shape[1]):
        sampled_path.append((arc[0,j]/cell_size, arc[1,j]/cell_size, ...))
```

## 6.2 Estaciones y Engranajes

**Lineas 30-47**:

```python
for i in range(1, len(path)):
    d = self.distance(path[i], path[i-1])
    stations[i] = stations[i-1] + d       # distancia acumulada

    # Determinar si avanza o retrocede
    d_row = path[i][0] - path[i-1][0]
    d_col = path[i][1] - path[i-1][1]
    avg_th = (path[i-1][2] + path[i][2]) / 2.0
    h_row = -math.sin(math.radians(avg_th))  # heading en coords grid
    h_col = math.cos(math.radians(avg_th))
    dot = d_row * h_row + d_col * h_col       # producto punto
    if dot >= 0: gears[i] = 1    # forward
    else:        gears[i] = -1   # reverse
```

- **Station**: distancia acumulada a lo largo del path.
- **Gear**: producto punto entre desplazamiento y heading. Positivo = forward, negativo = reverse.

\begin{questionbox}
"Por que usas -sin para el componente de fila?"
\end{questionbox}
\begin{answerbox}
En coordenadas de grid, la fila aumenta hacia ABAJO. En matematicas, y aumenta hacia arriba. Heading 90 grados (hacia "arriba") significa que la fila DISMINUYE. El heading en direccion de fila es $-\sin(\theta)$.
\end{answerbox}

## 6.3 Perfil de Velocidad Trapezoidal

![Perfiles de velocidad, aceleracion y velocidad angular](RMPC_Assignment2_ENTREGA/report_figures/fig5_velocity_profile.png){ width=55% }

Usa la ecuacion cinematica: $v_f^2 = v_i^2 + 2 \cdot a \cdot \Delta s$

### Forward Pass - Aceleracion (lineas 124-131):

```python
fwd = [0.0] * len(stations)
for i in range(1, len(stations)):
    ds = stations[i] - stations[i-1]
    if ds < 1e-9:
        fwd[i] = fwd[i-1]
    else:
        v2 = fwd[i-1]**2 + 2 * max_accel * ds
        fwd[i] = min(math.sqrt(max(v2, 0)), max_velocity)
```

Maxima velocidad alcanzable en cada punto **si el robot acelera desde reposo**.

### Backward Pass - Deceleracion (lineas 136-143):

```python
bwd = [0.0] * len(stations)
for i in range(len(stations) - 2, -1, -1):
    ds = stations[i+1] - stations[i]
    if ds < 1e-9:
        bwd[i] = bwd[i+1]
    else:
        v2 = bwd[i+1]**2 + 2 * max_accel * ds
        bwd[i] = min(math.sqrt(max(v2, 0)), max_velocity)
```

Misma ecuacion pero **hacia atras desde el final** (donde debe detenerse).

### Merge - Perfil Final (lineas 147-154):

```python
for i in range(len(stations)):
    profile[i] = min(fwd[i], bwd[i])    # minimo de ambas envolventes
```

Esto crea el perfil trapezoidal clasico: acelerar $\rightarrow$ crucero $\rightarrow$ decelerar.

\begin{questionbox}
"Explica el perfil de velocidad trapezoidal."
\end{questionbox}
\begin{answerbox}
El robot arranca en reposo, acelera a max\_acceleration hasta alcanzar max\_velocity (o hasta que necesite frenar), cruza a max\_velocity, y decelera para detenerse en el goal. Es el perfil \textbf{optimo en tiempo} bajo restricciones de aceleracion constante. El forward pass computa la envolvente "acelerando desde reposo", el backward pass computa "decelerando hasta reposo", y min() da el perfil factible.
\end{answerbox}

\begin{questionbox}
"De donde viene la ecuacion $v_f^2 = v_i^2 + 2as$?"
\end{questionbox}
\begin{answerbox}
De las ecuaciones cinematicas. Combinando $v_f = v_i + at$ y $s = v_i t + \frac{1}{2}at^2$, eliminando el tiempo $t$, se obtiene $v_f^2 = v_i^2 + 2as$.
\end{answerbox}

\begin{questionbox}
"Que pasa si el camino es muy corto para alcanzar max\_velocity?"
\end{questionbox}
\begin{answerbox}
Las envolventes forward y backward se cruzan antes de llegar a max\_velocity, creando un perfil \textbf{triangular} (acelera e inmediatamente decelera, sin fase de crucero). El min() lo maneja automaticamente.
\end{answerbox}

## 6.4 Velocidad y Velocidad Angular

**Lineas 86-95**:

```python
for i in range(1, nfe):
    dx = result.states[i].x - result.states[i-1].x
    dy = result.states[i].y - result.states[i-1].y
    result.states[i].v = math.sqrt(dx**2 + dy**2) / dt   # v = |desp| / dt
    dth = result.states[i].theta - result.states[i-1].theta
    result.states[i].omega = dth / dt                      # omega = dtheta/dt
if nfe > 1:
    result.states[0].v = result.states[1].v       # condicion de frontera
    result.states[0].omega = result.states[1].omega
```

Diferencias finitas hacia atras. Primer elemento se copia del segundo (no hay indice -1).

## 6.5 Aceleracion

**Lineas 99-102**:

```python
for i in range(1, nfe):
    result.states[i].a = (result.states[i].v - result.states[i-1].v) / dt
if nfe > 1:
    result.states[0].a = result.states[1].a
```

## 6.6 Angulos Continuos

**Lineas 258-269** - `to_continuous_angle()`:

```python
rads = []
for a in angles:
    rads.append(math.radians(a))     # grados -> radianes
out = [rads[0]]
for i in range(1, len(rads)):
    diff = rads[i] - rads[i-1]
    diff = self.normalize_angle(diff)    # normalizar a [-pi, pi]
    out.append(out[-1] + diff)           # acumular suavemente
return out
```

\begin{questionbox}
"Que hace to\_continuous\_angle y por que es necesaria?"
\end{questionbox}
\begin{answerbox}
Sin ella, interpolar entre heading 350 deg y 10 deg iria por 180, 90, 0 en vez de cruzar suavemente por 360/0 (+20 grados). La representacion continua "desenvuelve" los angulos para que la interpolacion no tenga saltos. Esto evita que el robot gire salvajemente.
\end{answerbox}

## 6.7 Normalize Angle

**Lineas 229-233**:

```python
while angle > math.pi:
    angle -= 2 * math.pi
while angle < -math.pi:
    angle += 2 * math.pi
return angle
```

Envuelve cualquier angulo al rango $[-\pi, \pi]$.

\newpage

# 7. VISUALIZACION DE RESULTADOS

## Trayectoria sobre el mapa

![Trayectoria generada sobre el mapa de obstaculos](RMPC_Assignment2_ENTREGA/report_figures/fig6_trajectory_on_map.png){ width=60% }

La trayectoria (linea verde) sigue el path del A\* interpolado, evitando obstaculos con curvas suaves.

## Trayectoria coloreada por velocidad

![Trayectoria coloreada por velocidad: oscuro = lento, claro = rapido](RMPC_Assignment2_ENTREGA/report_figures/fig7_trajectory_velocity_colored.png){ width=55% }

Colores oscuros = baja velocidad (inicio/fin, curvas). Colores claros/amarillos = alta velocidad (segmentos rectos, crucero).

## Perfil de orientacion

![Orientacion del robot a lo largo del tiempo](RMPC_Assignment2_ENTREGA/report_figures/fig8_orientation_profile.png){ width=80% }

Inicia en 90 grados, gira por las curvas, y termina cerca de 270 grados (heading del goal).

\newpage

# 8. CODIGO PRE-CONSTRUIDO QUE DEBES ENTENDER

## Clase Graph (`utils.py` lineas 6-31)

```python
class Graph:
    _vert_list = []              # Lista de vertices (row, col, angle)
    _edge_dict = {}              # (v1, v2) -> peso
    _adjacency_matrix            # Matriz NxN de pesos
```

**Nota**: La adjacency matrix tiene convencion transpuesta: `_adjacency_matrix[j, i] = edge_dict[(u, v)]`.

## ObstaclesGrid (`utils.py` lineas 33-89)

- `map`: Boolean numpy array (100x100). True = obstaculo.
- `is_edge_valid()`: Muestrea puntos a lo largo de una arista y verifica colisiones.
- `is_point_valid()`: Verifica que un punto no esta en obstaculo ni fuera del mapa.

## get\_neighbor (`lattice_planner.py` lineas 265-275)

```python
def get_neighbor(self, u, graph_vert_list, adjacency_matrix):
    row = graph_vert_list.index(u)
    is_adj = (adjacency_matrix[row, :] < np.inf) & \
             (adjacency_matrix[row, :] > 0)
    adj_list = []
    for i, v in enumerate(graph_vert_list):
        if is_adj[i]:
            adj_list.append(v)
    return adj_list
```

\newpage

# 9. BANCO DE PREGUNTAS RAPIDAS (FLASH CARDS)

| \# | Pregunta | Respuesta |
|----|----------|-----------|
| 1 | Que estructura de datos usa A\*? | Priority Queue (min-heap), set, dicts |
| 2 | Diferencia A\* vs Dijkstra? | A\* usa heuristica h(n). f = g + h |
| 3 | Que es "admisible"? | h nunca sobreestima el costo real |
| 4 | Que es "consistente"? | h(n) $\leq$ cost(n,n') + h(n') |
| 5 | Cuantos vertices en tu lattice? | 10 x 10 x 4 = 400 |
| 6 | Peso arista recta? | 1 |
| 7 | Peso arista arco? | $\pi$ |
| 8 | Como se bloquean obstaculos? | Peso = $\infty$ |
| 9 | Es RRT optimo? | No. Prob. completo, no optimo |
| 10 | Que es PRM? | Sample espacio libre, conectar, buscar |
| 11 | Fases de PRM? | Construccion + Consulta |
| 12 | Cuando PRM > RRT? | Multiples queries, mismo entorno |
| 13 | Perfil trapezoidal? | Acelerar $\rightarrow$ crucero $\rightarrow$ decelerar |
| 14 | $v_f^2 = v_i^2 + 2as$ viene de? | Eliminar t de $v=v_0+at$ y $s=v_0 t+\frac{1}{2}at^2$ |
| 15 | Por que forward Y backward? | min(acelerando, decelerando) = factible |
| 16 | Velocidad angular? | $\omega = d\theta/dt$ |
| 17 | Por que deg a rad? | Formulas fisicas usan radianes |
| 18 | normalize\_angle? | Envuelve a $[-\pi, \pi]$ |
| 19 | to\_continuous\_angle? | Desenvuelve para evitar saltos en 0/360 |
| 20 | Que es el gear? | Forward (1) o reverse (-1), por dot product |
| 21 | Que es una station? | Distancia acumulada a lo largo del path |
| 22 | Lattice cell size? | 10 unidades |
| 23 | Que pasa si no hay camino A\*? | Cola se vacia, retorna None |
| 24 | Complejidad A\*? | O(V log V + E log V) |
| 25 | Que es lazy deletion? | Push duplicados, skip stale al pop |
| 26 | Goal check: al push o al pop? | Al pop (garantiza optimalidad) |
| 27 | Si h no es admisible? | A\* puede dar camino suboptimo |
| 28 | Que es completitud probabilistica? | P(encontrar path) $\rightarrow$ 1 con iter $\rightarrow \infty$ |
| 29 | Diferencia collision check RRT vs PRM? | Misma idea, diferente densidad de muestreo |
| 30 | Adjacency matrix almacena? | Pesos. 0=sin arista, +val=peso, $\infty$=bloqueada |
