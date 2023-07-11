#ifndef ASTAR_H
#define ASTAR_H

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "../ciudad.h"
#include "../graph.h"

using namespace std;

template <typename TV, typename TE> struct HeuristicData {
  TV vertex;
  TE heuristic;
};

template <typename TV, typename TE> class AStar {
private:
  Graph<TV, TE> graph;
  unordered_map<string, TE> g_scores;
  unordered_map<string, TE> f_scores;
  unordered_map<string, string> came_from;

public:
  AStar(Graph<TV, TE> &g) : graph(g) {}

  void setGraph(const Graph<TV, TE> &g) { graph = g; }

  void setHeuristic(const unordered_map<string, TE> &heuristic_map) {
    for (auto &h_data : heuristic_map) {
      f_scores[h_data.first] = h_data.second;
    }
  }

  void clear() {
    g_scores.clear();
    f_scores.clear();
    came_from.clear();
  }

  vector<string> getPath(const string &end_vertex) {
    vector<string> path;
    stack<string> s;
    string current_vertex = end_vertex;
    s.push(current_vertex);

    while (came_from.find(current_vertex) != came_from.end()) {
      current_vertex = came_from[current_vertex];
      s.push(current_vertex);
    }
    ;
    while (!s.empty()) {
      path.push_back(s.top());
      s.pop();
    }
    path.erase(path.begin()); 
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

  void aStarSearch(const string &start_vertex, const string &end_vertex) {
    // Inicializar las puntuaciones G y F con un valor infinito y el nodo previo
    // como vacío
    for (const auto &vertex : graph.getVertices()) {
      g_scores[vertex.first] = numeric_limits<TE>::max();
      f_scores[vertex.first] = numeric_limits<TE>::max();
      came_from[vertex.first] = "";
    }

    // La puntuación G del vértice de inicio es 0
    g_scores[start_vertex] = 0;

    // Utilizar una cola de prioridad para obtener el vértice con la puntuación
    // F más baja en cada iteración
    priority_queue<pair<TE, string>, vector<pair<TE, string>>,
                   greater<pair<TE, string>>>
        pq;
    pq.push({f_scores[start_vertex], start_vertex});

    while (!pq.empty()) {
      string current_vertex = pq.top().second;
      pq.pop();

      if (current_vertex == end_vertex) {
        break; // Se alcanzó el vértice de destino, se ha encontrado el camino
               // más corto
      }

      // Obtener los vértices adyacentes
      list<Edge<TV, TE> *> edges = graph.getVertex(current_vertex)->edges;
      for (const auto &edge : edges) {
        Vertex<TV, TE> *neighbor_vertex =
            edge->vertexes[0] == graph.getVertex(current_vertex)
                ? edge->vertexes[1]
                : edge->vertexes[0];
        TE weight = edge->weight;

        // Calcular la puntuación G tentativa desde el vértice de inicio hasta
        // el vecino
        TE tentative_g_score = g_scores[current_vertex] + weight;

        if (tentative_g_score < g_scores[neighbor_vertex->data]) {
          // Se encontró un camino mejor hacia el vecino
          came_from[neighbor_vertex->data] = current_vertex;
          g_scores[neighbor_vertex->data] = tentative_g_score;
          f_scores[neighbor_vertex->data] =
              tentative_g_score + f_scores[neighbor_vertex->data];

          // Actualizar la cola de prioridad con la puntuación F actualizada
          pq.push({f_scores[neighbor_vertex->data], neighbor_vertex->data});
        }
      }
    }
  }
};

#endif