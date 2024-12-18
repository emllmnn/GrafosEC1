#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <set>
#include <ctime>
#include <string>
#include <memory>
#include <chrono>  
#include <climits>

template <typename K, typename V>
class Pair {
public:
    K key;
    V value;
    Pair(K key, V value) : key(key), value(value) {}

    K getKey() const {
        return key;
    }

    V getValue() const {
        return value;
    }
};

Pair<int, std::vector<Pair<int, int>>> readingFile(const std::string& fileName) {
    std::ifstream file(fileName);
    int num_vertices;
    file >> num_vertices;  
    std::vector<Pair<int, int>> arestas;
    std::string line;
    std::getline(file, line);  

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        int s, d;
        
        if (iss >> s) {
            if (iss >> d) {
                arestas.push_back(Pair<int, int>(s, d));
            } //else {
                //std::cout << "Vértice isolado: " << s << std::endl;
            //}
        } else {
            std::cout << "linha ignorada: " << line << std::endl;
        }
    }

    return Pair<int, std::vector<Pair<int, int>>>(num_vertices, arestas);
}

std::unordered_map<int, std::vector<int>> adjList(int num_vertices, const std::vector<Pair<int, int>>& arestas) {
    std::unordered_map<int, std::vector<int>> adj;
    for (int count = 1; count <= num_vertices; ++count) {
        adj[count] = std::vector<int>();
    }
    for (const auto& aresta : arestas) {
        adj[aresta.getKey()].push_back(aresta.getValue());
        adj[aresta.getValue()].push_back(aresta.getKey());
    }
    return adj;
}

std::vector<std::vector<bool>> adjMatrix(int num_vertices, const std::vector<Pair<int, int>>& arestas) {
    std::vector<std::vector<bool>> adj(num_vertices, std::vector<bool>(num_vertices, false));
    for (const auto& aresta : arestas) {
        adj[aresta.getKey() - 1][aresta.getValue() - 1] = true;
        adj[aresta.getValue() - 1][aresta.getKey() - 1] = true;
    }
    return adj;
}

std::unordered_set<int> bfsMatrix(const std::vector<std::vector<bool>>& adj, int start) {
    std::unordered_set<int> visited;
    std::queue<int> queue;
    queue.push(start);

    while (!queue.empty()) {
        int v = queue.front();
        queue.pop();
        if (visited.find(v) == visited.end()) {
            visited.insert(v);
            for (int neighbor = 0; neighbor < adj.size(); ++neighbor) {
                if (adj[v - 1][neighbor] && visited.find(neighbor + 1) == visited.end()) {
                    queue.push(neighbor + 1);
                }
            }
        }
    }

    return visited;
}

std::unordered_set<int> bfs(const std::unordered_map<int, std::vector<int>>& adj, int start) {
    if (adj.find(start) == adj.end()) {
        std::cout << "erro: o vértice de início " << start << " não está na lista de adjacência." << std::endl;
        return {};
    }

    std::unordered_set<int> visited;
    std::queue<int> queue;
    queue.push(start);
    while (!queue.empty()) {
        int v = queue.front();
        queue.pop();
        if (visited.find(v) == visited.end()) {
            visited.insert(v);
            for (int neighbor : adj.at(v)) {
                if (visited.find(neighbor) == visited.end()) {
                    queue.push(neighbor);
                }
            }
        }
    }

    return visited;
}

int firstVertex(const std::vector<std::vector<bool>>& adjMatrix) {
    for (int count = 0; count < adjMatrix.size(); ++count) {
        for (int j = 0; j < adjMatrix[count].size(); ++j) {
            if (adjMatrix[count][j]) {
                return count + 1; 
            }
        }
    }
    return -1; 
}

int bfsConex(const std::unordered_map<int, std::vector<int>>& adj, int start, std::unordered_set<int>& visited) {
    std::queue<int> queue;
    queue.push(start);
    visited.insert(start);
    int size = 0;
    while (!queue.empty()) {
        int v = queue.front();
        queue.pop();
        ++size; 
        for (int neighbor : adj.at(v)) {
            if (visited.find(neighbor) == visited.end()) {
                visited.insert(neighbor);
                queue.push(neighbor);
            }
        }
    }

    return size;
}

void findComponents(const std::unordered_map<int, std::vector<int>>& adj) {
    std::unordered_set<int> visited; 
    int num_components = 0;
    int max_component_size = 0;
    int min_component_size = INT_MAX;
    for (const auto& pair : adj) {
        int vertex = pair.first;
        if (visited.find(vertex) == visited.end()) {
            int component_size = bfsConex(adj, vertex, visited);
            num_components++;
            if (component_size > max_component_size) {
                max_component_size = component_size;
            }
            if (component_size < min_component_size) {
                min_component_size = component_size;
            }
        }
    }
    std::cout << "número de componentes conexos: " << num_components << std::endl;
    std::cout << "tamanho do maior componente: " << max_component_size << std::endl;
    std::cout << "tamanho do menor componente: " << (min_component_size == INT_MAX ? 0 : min_component_size) << std::endl;
}

template <typename T>
size_t memory(const T& obj) {
    return sizeof(obj);
}

int main() {
    std::string fileName = "collabGraph.txt"; 
    auto data = readingFile(fileName);
    int num_vertices = data.getKey();
    auto arestas = data.getValue();

    clock_t startTime = clock();
    auto adjListGraph = adjList(num_vertices, arestas);
    double timeAdjList = double(clock() - startTime) / CLOCKS_PER_SEC;

    startTime = clock();
    auto adjMatrixGraph = adjMatrix(num_vertices, arestas);
    double timeAdjMatrix = double(clock() - startTime) / CLOCKS_PER_SEC;

    int firstVertex = adjListGraph.begin()->first;

    using namespace std::chrono;
    auto startTimeM = std::chrono::high_resolution_clock::now();
    auto bfsResultList = bfs(adjListGraph, firstVertex);
    auto endTimeM = std::chrono::high_resolution_clock::now();
    auto timeBFSList = duration_cast<nanoseconds>(endTimeM - startTimeM);

    startTime = clock();
    auto bfsResultMatrix = bfsMatrix(adjMatrixGraph, firstVertex);
    double timeBFSMatrix = double(clock() - startTime) / CLOCKS_PER_SEC;

    size_t adjListMemory = memory(adjListGraph);
    size_t adjMatrixMemory = memory(adjMatrixGraph);

    std::cout << "tempo para construir lista de adjacência: " << timeAdjList << " segundos" << std::endl;
    std::cout << "tempo para construir matriz de adjacência: " << timeAdjMatrix << " segundos" << std::endl;
    std::cout << "tempo para BFS em lista: " << timeBFSList.count() << " nanosegundos" << std::endl;
    std::cout << "tempo para BFS em matriz: " << timeBFSMatrix << " segundos" << std::endl;
    std::cout << "tamanho da lista de adjacência: " << adjListMemory << " Bytes" << std::endl;
    std::cout << "tamanho da matriz de adjacência: " << adjMatrixMemory << " Bytes" << std::endl;

    findComponents(adjListGraph);

    return 0;
}

