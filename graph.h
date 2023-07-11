#ifndef GRAPH_H
#define GRAPH_H

#include <string>
#include <list>
#include <unordered_map>
#include <vector>
#include <iostream>

using namespace std;

template <typename TV, typename TE> struct Edge;

template <typename TV, typename TE> struct Vertex;

template <typename TV, typename TE> class Graph;

//////////////////////////////////////////////////////

template <typename TV, typename TE> struct Edge {
  Vertex<TV, TE> *vertexes[2];
  TE weight;
};

template <typename TV, typename TE> struct Vertex {
  TV data;
  list<Edge<TV, TE> *> edges;
};

template <typename TV, typename TE> class Graph {
private:
  unordered_map<string, Vertex<TV, TE>*> vertexes;

public:
  bool insertVertex(string id, TV vertex) {
    if (vertexes.find(id) != vertexes.end()) {
      return false; // El vértice ya existe
    }

    Vertex<TV, TE>* newVertex = new Vertex<TV, TE>();
    newVertex->data = vertex;

    vertexes[id] = newVertex;

    return true;
  }

  unordered_map<string, Vertex<TV, TE>*> getVertices() const {
    return vertexes;
  }

  Vertex<TV, TE>* getVertex(const string& id) {
      if (vertexes.find(id) == vertexes.end()) {
        return nullptr;
      }
  
      return vertexes[id];
    }

  bool createEdge(string id1, string id2, TE w) {
    // Verificar si los vértices existen
    if (vertexes.find(id1) == vertexes.end() || vertexes.find(id2) == vertexes.end()) {
      return false; // No se pueden crear aristas entre vértices inexistentes
    }

    Vertex<TV, TE>* vertex1 = vertexes[id1];
    Vertex<TV, TE>* vertex2 = vertexes[id2];

    // Verificar si la arista ya existe
    for (Edge<TV, TE>* edge : vertex1->edges) {
      if ((edge->vertexes[0] == vertex1 && edge->vertexes[1] == vertex2) ||
          (edge->vertexes[0] == vertex2 && edge->vertexes[1] == vertex1)) {
        return false; // La arista ya existe
      }
    }

    // Crear la arista
    Edge<TV, TE>* newEdge = new Edge<TV, TE>();
    newEdge->vertexes[0] = vertex1;
    newEdge->vertexes[1] = vertex2;
    newEdge->weight = w;

    vertex1->edges.push_back(newEdge);
    vertex2->edges.push_back(newEdge);

    return true;
  }

  bool deleteVertex(string id) {
    // Verificar si el vértice existe
    if (vertexes.find(id) == vertexes.end()) {
      return false; // No se puede eliminar un vértice inexistente
    }

    Vertex<TV, TE>* vertexToDelete = vertexes[id];

    // Eliminar todas las aristas conectadas al vértice
    for (Edge<TV, TE>* edge : vertexToDelete->edges) {
      Vertex<TV, TE>* neighborVertex = edge->vertexes[0] == vertexToDelete ? edge->vertexes[1] : edge->vertexes[0];
      neighborVertex->edges.remove(edge);
      delete edge;
    }

    // Eliminar el vértice
    vertexes.erase(id);

    delete vertexToDelete;

    return true;
  }

  bool deleteEdge(string id1, string id2) {
    // Verificar si los vértices existen
    if (vertexes.find(id1) == vertexes.end() || vertexes.find(id2) == vertexes.end()) {
      return false; // No se pueden eliminar aristas entre vértices inexistentes
    }

    Vertex<TV, TE>* vertex1 = vertexes[id1];
    Vertex<TV, TE>* vertex2 = vertexes[id2];

    Edge<TV, TE>* edgeToDelete = nullptr;

    // Buscar la arista a eliminar
    for (Edge<TV, TE>* edge : vertex1->edges) {
      if ((edge->vertexes[0] == vertex1 && edge->vertexes[1] == vertex2) ||
          (edge->vertexes[0] == vertex2 && edge->vertexes[1] == vertex1)) {
        edgeToDelete = edge;
        break;
      }
    }

    if (edgeToDelete == nullptr) {
      return false; // No se encontró la arista
    }

    // Eliminar la arista
    vertex1->edges.remove(edgeToDelete);
    vertex2->edges.remove(edgeToDelete);

    delete edgeToDelete;

    return true;
  }

  bool isConnected() {
    if (vertexes.empty()) {
      return false;
    }

    string startVertex = vertexes.begin()->first;

    std::unordered_map<string, bool> visited;
    DFS(startVertex, visited);

    for (const auto& vertex : vertexes) {
      if (!visited[vertex.first]) {
        return false;
      }
    }

    return true;
  }

  bool empty() {
    return vertexes.empty();
  }

  void clear() {
    for (const auto& vertex : vertexes) {
      deleteVertex(vertex.first);
    }

    vertexes.clear();
  }

  void display() {
    for (const auto& vertex : vertexes) {
      displayVertex(vertex.first);
    }
  }

  void displayVertex(string id) {
    // Verificar si el vértice existe
    if (vertexes.find(id) == vertexes.end()) {
      cout << "Vertex not found" << endl;
      return;
    }

    cout << "Vertex: " << id << endl;

    Vertex<TV, TE>* vertex = vertexes[id];

    cout << "Edges: ";
    for (Edge<TV, TE>* edge : vertex->edges) {
      Vertex<TV, TE>* neighborVertex = edge->vertexes[0] == vertex ? edge->vertexes[1] : edge->vertexes[0];
      cout << "(" << neighborVertex->data << ", " << edge->weight << ") ";
    }

    cout << endl;
  }

  TV findById(string id) {
    // Verificar si el vértice existe
    if (vertexes.find(id) == vertexes.end()) {
      throw std::out_of_range("Vertex not found");
    }

    return vertexes[id]->data;
  }

private:
  void DFS(string id, std::unordered_map<string, bool>& visited) {
    visited[id] = true;

    Vertex<TV, TE>* vertex = vertexes[id];

    for (Edge<TV, TE>* edge : vertex->edges) {
      Vertex<TV, TE>* neighborVertex = edge->vertexes[0] == vertex ? edge->vertexes[1] : edge->vertexes[0];

      if (!visited[neighborVertex->data]) {
        DFS(neighborVertex->data, visited);
      }
    }
  }
};

#endif
