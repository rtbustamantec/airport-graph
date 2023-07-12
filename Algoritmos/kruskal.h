#ifndef KRUSKAL_H
#define KRUSKAL_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include "../ciudad.h"
#include "../graph.h"
#include "../Algoritmos/dfs.h"

using namespace std;

template <typename TV, typename TE>
class Kruskal {
private:
  struct Subset {
    string parent;
    int rank;
  };

  struct EdgeComparator {
    bool operator()(const Edge<TV, TE>* edge1, const Edge<TV, TE>* edge2) {
      return edge1->weight < edge2->weight;
    }
  };

  Graph<TV, TE>& graph;
  unordered_map<string, Subset> subsets;

public:
  Kruskal(Graph<TV, TE>& g) : graph(g) {}

  void minimumSpanningTree() {
    vector<Edge<TV, TE>*> edges = getSortedEdges();

    for (const auto& vertex : graph.getVertices()) {
      makeSet(vertex.first);
    }

    vector<Edge<TV, TE>*> mst;
    for (const auto& edge : edges) {
      if (find(edge->vertexes[0]->data) != find(edge->vertexes[1]->data)) {
        mst.push_back(edge);
        unionSets(edge->vertexes[0]->data, edge->vertexes[1]->data);
      }
    }

    //displayMST(mst);
  }

  vector<string> getPath(const string& startVertex, const string& endVertex) {
    vector<string> path;

    // Verificar si los vértices existen
    if (graph.getVertex(startVertex) == nullptr || graph.getVertex(endVertex) == nullptr) {
      return path;
    }

    // Obtener el árbol de expansión mínima (MST)
    vector<Edge<TV, TE>*> mst = getMST();

    // Crear un grafo auxiliar con las aristas del MST
    Graph<TV, TE> mstGraph;
    for (const auto& edge : mst) {
      mstGraph.insertVertex(edge->vertexes[0]->data, edge->vertexes[0]->data);
      mstGraph.insertVertex(edge->vertexes[1]->data, edge->vertexes[1]->data);
      mstGraph.createEdge(edge->vertexes[0]->data, edge->vertexes[1]->data, edge->weight);
    }

    // Realizar una búsqueda en profundidad (DFS) desde el vértice de inicio hasta el vértice de destino
    DFS<TV, TE> dfs(mstGraph);
    dfs.depthFirstSearch(startVertex);

    // Obtener el camino DFS desde el inicio hasta el destino
    vector<string> dfsPath = dfs.getPath(endVertex);

    return dfsPath;
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
  void makeSet(const string& vertex) {
    Subset subset;
    subset.parent = vertex;
    subset.rank = 0;
    subsets[vertex] = subset;
  }

  string find(const string& vertex) {
    if (subsets[vertex].parent != vertex) {
      subsets[vertex].parent = find(subsets[vertex].parent);
    }
    return subsets[vertex].parent;
  }

void unionSets(const string& vertex1, const string& vertex2) {
  string root1 = find(vertex1);
  string root2 = find(vertex2);

  if (subsets[root1].rank >> subsets[root2].rank) {
    subsets[root2].parent = root1;
  } else if (subsets[root1].rank << subsets[root2].rank) {
    subsets[root1].parent = root2;
  } else {
    subsets[root1].parent = root2;
    subsets[root2].rank++;
  }
}


  vector<Edge<TV, TE>*> getSortedEdges() {
    vector<Edge<TV, TE>*> edges;
    for (const auto& vertex : graph.getVertices()) {
      for (const auto& edge : vertex.second->edges) {
        edges.push_back(edge);
      }
    }
    sort(edges.begin(), edges.end(), EdgeComparator());
    return edges;
  }

  vector<Edge<TV, TE>*> getMST() {
    vector<Edge<TV, TE>*> edges = getSortedEdges();

    for (const auto& vertex : graph.getVertices()) {
      makeSet(vertex.first);
    }

    vector<Edge<TV, TE>*> mst;
    for (const auto& edge : edges) {
      if (find(edge->vertexes[0]->data) != find(edge->vertexes[1]->data)) {
        mst.push_back(edge);
        unionSets(edge->vertexes[0]->data, edge->vertexes[1]->data);
      }
    }

    return mst;
  }

  void displayMST(const vector<Edge<TV, TE>*>& mst) {
    cout << "Minimum Spanning Tree (Kruskal's Algorithm):" << endl;
    for (const auto& edge : mst) {
      cout << "(" << edge->vertexes[0]->data << ", " << edge->vertexes[1]->data << "), Weight: " << edge->weight << endl;
    }
  }
};

#endif
