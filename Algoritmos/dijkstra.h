#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include "../ciudad.h"
#include "../graph.h"

using namespace std;
template<typename TV, typename TE>
class Dijkstra {
private:
  Graph<TV, TE> graph;
  unordered_map<string, TE> dist;
  unordered_map<string, string> prev;
  
public:
  Dijkstra(Graph<TV, TE>& g) : graph(g) {}
  
  void shortestPath(const string& startVertex) {
    // Inicializar las distancias con un valor infinito y el nodo previo como vacío
    for (const auto& vertex : graph.getVertices()) {
      dist[vertex.first] = numeric_limits<TE>::max();
      prev[vertex.first] = "";
    }
    
    // La distancia al nodo de inicio es 0
    dist[startVertex] = 0;
    
    // Utilizar una cola de prioridad para obtener el vértice con la distancia mínima en cada iteración
    std::priority_queue<std::pair<TE, std::string>, std::vector<std::pair<TE, std::string>>, std::greater<std::pair<TE, std::string>>> pq;
    pq.push({0, startVertex});
    
    while (!pq.empty()) {
      std::string currentVertex = pq.top().second;
      pq.pop();
      
      // Obtener los vértices adyacentes
      std::list<Edge<TV, TE>*> edges = graph.getVertex(currentVertex)->edges;
      for (const auto& edge : edges) {
        Vertex<TV, TE>* neighborVertex;
        if (edge->vertexes[0] == graph.getVertex(currentVertex)) {
          neighborVertex = edge->vertexes[1];
        } else {
          neighborVertex = edge->vertexes[0];
        }
        TE weight = edge->weight;
        
        // Calcular la distancia tentativa desde el vértice de inicio hasta el vecino
        TE tentativeDist = dist[currentVertex] + weight;
        if (tentativeDist < dist[neighborVertex->data]) {
          dist[neighborVertex->data] = tentativeDist;
          prev[neighborVertex->data] = currentVertex;
          pq.push({tentativeDist, neighborVertex->data});
        }
      }
    }
  }


  
  std::vector<std::string> getPath(const std::string& endVertex) {
    std::vector<std::string> path;
    
    // Reconstruir el camino desde el nodo final hasta el nodo de inicio
    std::string currentVertex = endVertex;
    while (prev[currentVertex] != "") {
      path.insert(path.begin(), currentVertex);
      currentVertex = prev[currentVertex];
    }
    
    if (path.size() > 0) {
      path.insert(path.begin(), currentVertex); // Agregar el nodo de inicio al camino
    }
    
    return path;
  }
  
  void displayPath(const std::vector<std::string> path, unordered_map<string, City*> ciudadHashtable) {
    if (path.size() == 0) {
      std::cout << "No existe un camino desde el origen hasta el destino." << std::endl;
    } else {
      for (const auto& vertex : path) {
        City* city = ciudadHashtable[vertex];
        std::cout << city->name << " - "<< city->city <<  " (ID: " << vertex << ") " << " -> ";
      }
      std::cout << "Fin" << std::endl;
    }
  }
};

#endif
