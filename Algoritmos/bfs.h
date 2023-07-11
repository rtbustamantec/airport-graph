#ifndef BFS_H
#define BFS_H

#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <vector>
#include "../ciudad.h"
#include "../graph.h"

using namespace std;

template<typename TV, typename TE>
class BFS {
private:
  Graph<TV, TE>& graph;
  unordered_map<string, string> prev;
  
public:
  BFS(Graph<TV, TE>& g) : graph(g) {}
  
  void breadthFirstSearch(const string& startVertex, const string& endVertex) {
    unordered_set<string> visited;
    queue<string> q;
    
    // Marcar el vértice de inicio como visitado y agregarlo a la cola
    visited.insert(startVertex);
    q.push(startVertex);
    
    while (!q.empty()) {
      string currentVertex = q.front();
      q.pop();
      
      // Verificar si se ha alcanzado el vértice de destino
      if (currentVertex == endVertex) {
        break;
      }
      
      // Obtener los vértices adyacentes al vértice actual
      list<Edge<TV, TE>*> edges = graph.getVertex(currentVertex)->edges;
      for (const auto& edge : edges) {
        Vertex<TV, TE>* neighborVertex;
        if (edge->vertexes[0] == graph.getVertex(currentVertex)) {
          neighborVertex = edge->vertexes[1];
        } else {
          neighborVertex = edge->vertexes[0];
        }
        TE weight = edge->weight;
        
        // Verificar si el vértice adyacente aún no ha sido visitado
        if (visited.find(neighborVertex->data) == visited.end()) {
          visited.insert(neighborVertex->data);
          q.push(neighborVertex->data);
          prev[neighborVertex->data] = currentVertex;
        }
      }
    }
    
  }
  
  vector<string> getPath(const string& startVertex, const string& endVertex) {
    vector<string> path;
    
    // Reconstruir el camino desde el vértice de destino hasta el vértice de inicio
    string currentVertex = endVertex;
    while (currentVertex != startVertex) {
      path.insert(path.begin(), currentVertex);
      currentVertex = prev[currentVertex];
    }
    
    // Agregar el vértice de inicio al camino
    path.insert(path.begin(), startVertex);
    
    return path;
  }
  
  void displayPath(const vector<string>& path, unordered_map<string, City*> ciudadHashtable) {
   if (path.size() == 0) {
      std::cout << "No existe un camino desde el origen hasta el destino." << std::endl;
    } else {
      std::cout << "Camino más corto: ";
      for (const auto& vertex : path) {
        City* city = ciudadHashtable[vertex];
        std::cout << city->name << " - "<< city->city <<  " (ID: " << vertex << ") " << " -> ";
      }
      std::cout << "Fin" << std::endl;
   }
  }
};

#endif
