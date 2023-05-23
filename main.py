import networkx as nx
import matplotlib.pyplot as plt

# Função que implementa o algoritmo de Dijkstra para encontrar o caminho mais curto
def dijkstra(grafo, inicio):
    # Inicializa todas as distâncias como infinito e a distância do ponto de partida como 0
    distancias = {vertice: float('infinity') for vertice in grafo}
    distancias[inicio] = 0

    # Inicializa todos os caminhos como vazio e o caminho do ponto de partida como o próprio ponto de partida
    caminhos = {vertice: [] for vertice in grafo}
    caminhos[inicio] = [inicio]

    # Cria uma lista de todos os vértices que ainda não foram visitados
    vertices_nao_visitados = list(grafo.keys())

    # Enquanto houver vértices não visitados
    while vertices_nao_visitados:
        # Encontra o vértice com a menor distância
        vertice_atual = min(
            [(vertice, distancias[vertice]) for vertice in vertices_nao_visitados],
            key=lambda x: x[1]
        )[0]

        # Para cada vizinho do vértice atual, calcula a nova distância proposta
        for vizinho, peso in grafo[vertice_atual].items():
            distancia_proposta = distancias[vertice_atual] + peso
            # Se a nova distância proposta é menor que a distância atual, atualiza a distância e o caminho
            if distancia_proposta < distancias[vizinho]:
                distancias[vizinho] = distancia_proposta
                caminhos[vizinho] = caminhos[vertice_atual] + [vizinho]

        # Remove o vértice atual dos vértices não visitados
        vertices_nao_visitados.remove(vertice_atual)

    # Retorna as distâncias e os caminhos
    return distancias, caminhos


# Função para desenhar o grafo
def draw_graph(graph, paths, start):
    # Cria um grafo direcionado
    G = nx.DiGraph()

    # Inicializa os rótulos das arestas como vazio
    edge_labels = {}

    # Para cada nó no grafo, adiciona uma aresta para cada vizinho com o respectivo peso
    for node in graph:
        for neighbor, weight in graph[node].items():
            G.add_edge(node, neighbor, weight=weight)
            edge_labels[(node, neighbor)] = weight

    # Usa o layout shell para a visualização do grafo
    shell_layout = nx.shell_layout(G)

    # Aumenta o tamanho da figura
    plt.figure(figsize=(7, 7))

    # Desenha os nós do grafo
    nx.draw_networkx_nodes(G, shell_layout, cmap=plt.get_cmap('jet'), node_size=400)
    # Desenha o nó de partida com uma cor diferente
    nx.draw_networkx_nodes(G, shell_layout, nodelist=[start], node_color='green', node_size=500)
    # Desenha os rótulos dos nós
    nx.draw_networkx_labels(G, shell_layout)
    # Desenha as arestas
    nx.draw_networkx_edges(G, shell_layout, edge_color='gray', arrows=True)
    # Desenha os rótulos das arestas
    nx.draw_networkx_edge_labels(G, shell_layout, edge_labels=edge_labels)

    # Para cada caminho, desenha as arestas do caminho em uma cor diferente
    for path in paths.values():
        edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        nx.draw_networkx_edges(G, shell_layout, edgelist=edges, edge_color='red', arrows=True)

    # Mostra o grafo
    plt.show()


grafo1 = {
    'A': {'B': 5, 'C': 7, 'D': 1},
    'B': {'C': 2},
    'C': {'E': 5, 'D': 6},
    'D': {'B': 3, 'F': 5, 'G': 3},
    'E': {'F': 4},
    'F': {'C': 1},
    'G': {'E': 1},
}

grafo2 = {
    'A': {'E': 5, 'F': 1, 'L': 2},
    'B': {'C': 11, 'I': 9},
    'C': {'D': 3, 'G': 5, 'J': 6, 'F': 3},
    'D': {'N': 5},
    'E': {'B': 1, 'H': 8},
    'F': {'I': 6, 'M': 4},
    'G': {'F': 1, 'D': 4},
    'H': {'M': 7},
    'I': {'H': 10},
    'J': {'K': 13, 'L': 8},
    'K': {},
    'L': {},
    'M': {'K': 9},
    'N': {'K': 6},
}

distancias, caminhos = dijkstra(grafo1, 'A')
print("Grafo 1")
print("Distâncias:", distancias)
print("Caminhos:", caminhos)
draw_graph(grafo1, caminhos, 'A')

print()

distancias, caminhos = dijkstra(grafo2, 'B')
print("Grafo 2")
print("Distâncias:", distancias)
print("Caminhos:", caminhos)
draw_graph(grafo2, caminhos, 'B')
