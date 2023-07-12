#ifndef DFS_H
#define DFS_H

#include <vector>
#include <unordered_map>
#include "../ciudad.h"
#include "../graph.h"


using namespace std;

template <typename TV, typename TE>
class DFS {
private:
  Graph<TV, TE>& graph;
  unordered_map<string, bool> visited;
  unordered_map<string, string> prev;

public:
  DFS(Graph<TV, TE>& g) : graph(g) {}

  void depthFirstSearch(const string& startVertex) {
    visited.clear();
    prev.clear();
    for (const auto& vertex : graph.getVertices()) {
      visited[vertex.first] = false;
      prev[vertex.first] = "";
    }

    dfsHelper(startVertex);
  }

  vector<string> getPath(const string& endVertex) {
    vector<string> path;

    if (prev[endVertex] == "") {
      return path; // No hay un camino desde el inicio hasta el destino
    }

    string currentVertex = endVertex;
    while (currentVertex != "") {
      path.insert(path.begin(), currentVertex);
      currentVertex = prev[currentVertex];
    }

    return path;
  }

  void displayPath(const vector<string>& path, unordered_map<string, City*>& ciudadHashtable) {
    if (path.empty()) {
      cout << "No existe un camino desde el origen hasta el destino." << endl;
    } else {
      for (const auto& vertex : path) {
        City* city = ciudadHashtable[vertex];
        cout << city->name << " - " << city->city << " (ID: " << vertex << ") -> ";
      }
      cout << "Fin" << endl;
    }
  }

private:
  void dfsHelper(const string& currentVertex) {
    visited[currentVertex] = true;
    list<Edge<TV, TE>*> edges = graph.getVertex(currentVertex)->edges;

    for (const auto& edge : edges) {
      Vertex<TV, TE>* neighborVertex;
      if (edge->vertexes[0] == graph.getVertex(currentVertex)) {
        neighborVertex = edge->vertexes[1];
      } else {
        neighborVertex = edge->vertexes[0];
      }

      if (!visited[neighborVertex->data]) {
        prev[neighborVertex->data] = currentVertex;
        dfsHelper(neighborVertex->data);
      }
    }
  }
};

#endif
