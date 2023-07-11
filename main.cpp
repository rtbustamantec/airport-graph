#include <iostream>
#include <chrono>
#include "json.hpp"
#include "graph.h"
#include <iostream>
#include <fstream>
#include "string"
#include "Algoritmos/dijkstra.h"
#include "Algoritmos/astar.h"
#include "Algoritmos/bfs.h"
#include "Algoritmos/dfs.h"
#include "Algoritmos/kruskal.h"

using json = nlohmann::json;

string getCityString(const json& data) {
  return data.dump();
}

City* getCityFromString(string city_string) {
  auto ciudad = json::parse(city_string);
  
  City* new_city = new City();
  new_city->name = ciudad["Name"];
  new_city->city = ciudad["City"];
  new_city->country = ciudad["Country"];
  new_city->airport_id = ciudad["Airport ID"];
  for(const auto& destino : ciudad["destinations"]){
     new_city->destinations.push_back(destino);
  }

  new_city->longitude = stod(ciudad["Latitude"].get<string>());
  new_city->latitude = stod(ciudad["Longitude"].get<string>());
  return new_city;
}

City* getCityFromJSON(const json& data) {
  City* new_city = new City();
  new_city->name = data["Name"];
  new_city->city = data["City"];
  new_city->country = data["Country"];
  new_city->airport_id = data["Airport ID"];
  for(const auto& destino : data["destinations"]){
     new_city->destinations.push_back(destino);
  }

  new_city->longitude = stod(data["Latitude"].get<string>());
  new_city->latitude = stod(data["Longitude"].get<string>());
  return new_city;
}

double calcularDistanciaEuclidiana(double latitud1, double longitud1, double latitud2, double longitud2) {
    // Convertir las latitudes y longitudes de grados a radianes
    double distancia = sqrt(pow(latitud1 - latitud2, 2) + pow(longitud1 - longitud2, 2));
    return distancia;
}



void insertarVertice(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string airportID;
  string airportName;
  string airportCity;
  string airportCountry;

  cout << "Ingrese el ID del aeropuerto: ";
  cin >> airportID;
  cout << "Ingrese el nombre del aeropuerto: ";
  cin.ignore();
  getline(cin, airportName);
  cout << "Ingrese la ciudad del aeropuerto: ";
  getline(cin, airportCity);
  cout << "Ingrese el país del aeropuerto: ";
  getline(cin, airportCountry);

  City* new_city = new City();
  new_city->name = airportName;
  new_city->city = airportCity;
  new_city->country = airportCountry;
  new_city->airport_id = airportID;
  new_city->longitude = 0.0;
  new_city->latitude = 0.0;

  ciudadHashtable[airportID] = new_city;
  graph.insertVertex(airportID, airportID);

  cout << "Vértice insertado exitosamente." << endl;
}

void eliminarVertice(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string airportID;

  cout << "Ingrese el ID del aeropuerto a eliminar: ";
  cin >> airportID;

  if (ciudadHashtable.count(airportID) > 0) {
    graph.deleteVertex(airportID);
    delete ciudadHashtable[airportID];
    ciudadHashtable.erase(airportID);
    cout << "Vértice eliminado exitosamente." << endl;
  } else {
    cout << "No se encontró el vértice con el ID especificado." << endl;
  }
}


void eliminarArista(Graph<string, int>& graph) {
  string sourceID;
  string destinationID;

  cout << "Ingrese el ID del vértice de origen: ";
  cin >> sourceID;
  cout << "Ingrese el ID del vértice de destino: ";
  cin >> destinationID;

  graph.deleteEdge(sourceID, destinationID);

  cout << "Arista eliminada exitosamente." << endl;
}

void insertarArista(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string airportID1;
  string airportID2;

  cout << "Ingrese el ID del primer aeropuerto: ";
  cin >> airportID1;
  cout << "Ingrese el ID del segundo aeropuerto: ";
  cin >> airportID2;

  if (ciudadHashtable.count(airportID1) > 0 && ciudadHashtable.count(airportID2) > 0) {
    double lat_1 = ciudadHashtable[airportID1]->latitude;
    double lon_1 = ciudadHashtable[airportID1]->longitude;
    double lat_2 = ciudadHashtable[airportID2]->latitude;
    double lon_2 = ciudadHashtable[airportID2]->longitude;
    double distancia = calcularDistanciaEuclidiana(lat_1, lon_1, lat_2, lon_2);

    graph.createEdge(airportID1, airportID2, distancia);
    cout << "Arista insertada exitosamente." << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void ejecutarDijkstra(Graph<string, int>& graph,unordered_map<string, City*> ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el vértice de inicio: ";
  cin >> start_vertex;
  cout << "Ingrese el vértice de destino: ";
  cin >> end_vertex;

  // Verificar si ambos vértices existen en el grafo
  bool startExists = false;
  bool endExists = false;

  for (const auto& vertex : graph.getVertices()) {
    if (vertex.first == start_vertex) {
      startExists = true;
    }
    if (vertex.first == end_vertex) {
      endExists = true;
    }
  }

  if (startExists && endExists) {
    // Medir el tiempo de ejecución
    clock_t start = clock();

    // Crear un objeto Dijkstra y calcular la ruta más corta desde el vértice de inicio
     Dijkstra<string, int> dijkstra(graph);
     dijkstra.shortestPath(start_vertex); // Reemplaza "start_vertex" con el vértice de inicio deseado
    // Obtener el camino más corto
     vector<string> path = dijkstra.getPath(end_vertex); // Reemplaza "end_vertex" con el vértice de destino deseado
  
    // Mostrar el camino más corto
    dijkstra.displayPath(path, ciudadHashtable);

    // Calcular el tiempo de ejecución
    clock_t end = clock();
    double duration = (end - start) / (double)CLOCKS_PER_SEC;

    cout << "Tiempo de ejecución: " << duration << " segundos" << endl;
  } else {
    cout << "Al menos uno de los vértices especificados no existe." << endl;
  }
}


void aStarAlgoritmo(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el ID del aeropuerto de origen: ";
  cin >> start_vertex;
  cout << "Ingrese el ID del aeropuerto de destino: ";
  cin >> end_vertex;

  if (ciudadHashtable.count(start_vertex) > 0 && ciudadHashtable.count(end_vertex) > 0) {
    AStar<string, int> a_star(graph);

    // Definir la heurística (distancia estimada desde cada vértice hasta el objetivo)
    unordered_map<string, int> heuristic;
    double dest_lat = ciudadHashtable[end_vertex]->latitude; 
    double dest_lon = ciudadHashtable[end_vertex]->longitude;

    for (const auto& vertex : graph.getVertices()) {
        Vertex<string, int>* current_vertex = vertex.second;
        double curr_lat = ciudadHashtable[current_vertex->data]->latitude; 
        double curr_lon = ciudadHashtable[current_vertex->data]->longitude; 

        double distance = calcularDistanciaEuclidiana(curr_lat, curr_lon, dest_lat, dest_lon);

        heuristic[current_vertex->data] = distance;
    }

    a_star.setHeuristic(heuristic);

    clock_t start = clock();
    a_star.aStarSearch(start_vertex, end_vertex);
    clock_t end = clock();
    double duration = (end - start) / (double)CLOCKS_PER_SEC;

    vector<string> a_path = a_star.getPath(end_vertex);

    cout << "Camino más corto por A*: ";
    for (const auto& vertex : a_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   " << endl;

    cout << "Tiempo de ejecución: " << duration << " segundos" << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void bfsAlgoritmo(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el ID del aeropuerto de origen: ";
  cin >> start_vertex;
  cout << "Ingrese el ID del aeropuerto de destino: ";
  cin >> end_vertex;

  if (ciudadHashtable.count(start_vertex) > 0 && ciudadHashtable.count(end_vertex) > 0) {
    BFS<string, int> bfs(graph);

    clock_t start = clock();
    bfs.breadthFirstSearch(start_vertex, end_vertex);
    clock_t end = clock();
    double duration = (end - start) / (double)CLOCKS_PER_SEC;

    vector<string> bfs_path = bfs.getPath(start_vertex, end_vertex);

    cout << "Camino más corto por BFS: ";
    for (const auto& vertex : bfs_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   " << endl;

    cout << "Tiempo de ejecución: " << duration << " segundos" << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void dfsAlgoritmo(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el ID del aeropuerto de origen: ";
  cin >> start_vertex;
  cout << "Ingrese el ID del aeropuerto de destino: ";
  cin >> end_vertex;

  if (ciudadHashtable.count(start_vertex) > 0 && ciudadHashtable.count(end_vertex) > 0) {
    DFS<string, int> dfs(graph);

    clock_t start = clock();
    dfs.depthFirstSearch(start_vertex);
    clock_t end = clock();
    double duration = (end - start) / (double)CLOCKS_PER_SEC;

    vector<string> dfsPath = dfs.getPath(end_vertex);

    cout << "Camino más corto por DFS: ";
    for (const auto& vertex : dfsPath) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   " << endl;

    cout << "Tiempo de ejecución: " << duration << " segundos" << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void kruskalAlgoritmo(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el ID del aeropuerto de origen: ";
  cin >> start_vertex;
  cout << "Ingrese el ID del aeropuerto de destino: ";
  cin >> end_vertex;

  if (ciudadHashtable.count(start_vertex) > 0 && ciudadHashtable.count(end_vertex) > 0) {
    Kruskal<string, int> kruskal(graph);

    clock_t start = clock();
    kruskal.minimumSpanningTree();
    clock_t end = clock();
    double duration = (end - start) / (double)CLOCKS_PER_SEC;

    vector<string> kruskalPath = kruskal.getPath(start_vertex, end_vertex);

    cout << "Camino más corto por Kruskal: ";
    for (const auto& vertex : kruskalPath) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   " << endl;

    cout << "Tiempo de ejecución: " << duration << " segundos" << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void resumenDistancias(Graph<string, int>& graph, unordered_map<string, City*>& ciudadHashtable) {
  string start_vertex;
  string end_vertex;

  cout << "Ingrese el ID del aeropuerto de origen: ";
  cin >> start_vertex;
  cout << "Ingrese el ID del aeropuerto de destino: ";
  cin >> end_vertex;

  if (ciudadHashtable.count(start_vertex) > 0 && ciudadHashtable.count(end_vertex) > 0) {
    Dijkstra<string, int> dijkstra(graph);
    AStar<string, int> a_star(graph);
    BFS<string, int> bfs(graph);
    DFS<string, int> dfs(graph);
    Kruskal<string, int> kruskal(graph);

    clock_t start;
    clock_t end;
    double duration;

    cout << "Resumen de distancias:" << endl;

    // Dijkstra
    start = clock();
    dijkstra.shortestPath(start_vertex);
    vector<string> dijkstra_path = dijkstra.getPath(end_vertex);
    end = clock();
    duration = (end - start) / (double)CLOCKS_PER_SEC;
    cout << "Dijkstra: ";
    for (const auto& vertex : dijkstra_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   ";
    cout << "(Tiempo: " << duration << " segundos)" << endl;

    // A*
    start = clock();
    a_star.aStarSearch(start_vertex, end_vertex);
    vector<string> a_star_path = a_star.getPath(end_vertex);
    end = clock();
    duration = (end - start) / (double)CLOCKS_PER_SEC;
    cout << "A*: ";
    for (const auto& vertex : a_star_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   ";
    cout << "(Tiempo: " << duration << " segundos)" << endl;

    // BFS
    start = clock();
    bfs.breadthFirstSearch(start_vertex, end_vertex);
    vector<string> bfs_path = bfs.getPath(start_vertex, end_vertex);
    end = clock();
    duration = (end - start) / (double)CLOCKS_PER_SEC;
    cout << "BFS: ";
    for (const auto& vertex : bfs_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   ";
    cout << "(Tiempo: " << duration << " segundos)" << endl;

    // DFS
    start = clock();
    dfs.depthFirstSearch(start_vertex);
    vector<string> dfs_path = dfs.getPath(end_vertex);
    end = clock();
    duration = (end - start) / (double)CLOCKS_PER_SEC;
    cout << "DFS: ";
    for (const auto& vertex : dfs_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   ";
    cout << "(Tiempo: " << duration << " segundos)" << endl;

    // Kruskal
    start = clock();
    kruskal.minimumSpanningTree();
    vector<string> kruskal_path = kruskal.getPath(start_vertex, end_vertex);
    end = clock();
    duration = (end - start) / (double)CLOCKS_PER_SEC;
    cout << "Kruskal: ";
    for (const auto& vertex : kruskal_path) {
      cout << vertex << " -> ";
    }
    cout << "\b\b\b   ";
    cout << "(Tiempo: " << duration << " segundos)" << endl;
  } else {
    cout << "No se encontraron los vértices especificados." << endl;
  }
}

void mostrarGrafo(Graph<string, int>& graph) {
  cout << "Grafo actualizado:" << endl;
  graph.display();
}




json leerArchivoJSON(const string& filename) {
  ifstream file("data/" + filename);  // Reemplaza "data/" con la ruta correcta a tu directorio de archivos JSON
  json data;

  if (!file) {
    cout << "Error al abrir el archivo JSON" << endl;
  } else {
    file >> data;
    file.close();
  }

  return data;
}

void mostrarMenu() {
  std::cout << "2. Insertar un vértice" << std::endl;
  std::cout << "3. Eliminar un vértice" << std::endl;
  std::cout << "4. Eliminar una arista" << std::endl;
  std::cout << "5. Insertar una arista" << std::endl;
  std::cout << "6. Seleccionar la distancia más corta por Dijkstra" << std::endl;
  std::cout << "7. Seleccionar la distancia más corta por A*" << std::endl;
  std::cout << "8. Seleccionar la distancia más corta por BFS" << std::endl;
  std::cout << "9. Seleccionar la distancia más corta por DFS" << std::endl;
  std::cout << "10. Seleccionar la distancia más corta por Kruskal" << std::endl;
  std::cout << "11. Resumen de las 5 distancias con tiempos de ejecución" << std::endl;
  std::cout << "12. Mostrar el grafo actualizado" << std::endl;
  std::cout << "0. Salir" << std::endl;
}



int main() {
  std::string filename;
  std::string filename2;
  Graph<std::string, int> graph;
  std::unordered_map<std::string, City*> ciudadHashtable;

  std::cout << "---- Menú ----" << std::endl;
  std::cout << "1. Seleccionar archivo JSON" << std::endl;
  std::cout << "0. Salir" << std::endl;
  std::cout << "Seleccione una opción: ";
  std::cin >> filename;

  if (filename == "0") {
    std::cout << "Saliendo del programa..." << std::endl;
    return 0;
  }
  cout << "Ingrese el nombre del archivo JSON (airports.json o pe.json): ";
  std::cin >> filename2;
  json data = leerArchivoJSON(filename2);
  
  // Insertar los vértices
  for (const auto& ciudad : data) {
    // string city_string = getCityString(ciudad);
    graph.insertVertex(ciudad["Airport ID"], ciudad["Airport ID"]);
    // graph.insertVertex(ciudad["Airport ID"], city_string);
    ciudadHashtable[ciudad["Airport ID"]] = getCityFromJSON(ciudad);
  }

  // Crear las aristas
  for (const auto& airport : data) {
    std::string airportID = airport["Airport ID"];
    json destinations = airport["destinations"];
    for (const auto& destinationID : destinations) {
      auto iterador = ciudadHashtable.find(destinationID);
      if (iterador != ciudadHashtable.end())  {
        City* ciudad_destino = ciudadHashtable[destinationID];
        double lat_1 = std::stod(airport["Latitude"].get<std::string>());
        double lon_1 = std::stod(airport["Longitude"].get<std::string>());
        double lat_2 = ciudad_destino->latitude;
        double lon_2 = ciudad_destino->longitude;
        double distancia = calcularDistanciaEuclidiana(lat_1, lon_1, lat_2, lon_2);
        graph.createEdge(airportID, destinationID, distancia);
      }
    }
  }
  
  if (data.is_null()) {
    return 0;
  }

    bool salir = false;
  while (!salir) {
    mostrarMenu();
    std::cout << "Seleccione una opción: ";
    int menu_option;
    std::cin >> menu_option;

    switch (menu_option) {
      case 0:
        std::cout << "Saliendo del programa..." << std::endl;
        salir = true;
        break;
      case 2:
        insertarVertice(graph, ciudadHashtable);
        break;
      case 3:
        eliminarVertice(graph, ciudadHashtable);
        break;
      case 4:
        eliminarArista(graph);
        break;
      case 5:
        insertarArista(graph, ciudadHashtable);
        break;
      case 6:
        ejecutarDijkstra(graph, ciudadHashtable);
        break;
      case 7:
        aStarAlgoritmo(graph, ciudadHashtable);
        break;
      case 8:
        bfsAlgoritmo(graph, ciudadHashtable);
        break;
      case 9:
        dfsAlgoritmo(graph, ciudadHashtable);
        break;
      case 10:
        kruskalAlgoritmo(graph, ciudadHashtable);
        break;
      case 11:
        resumenDistancias(graph, ciudadHashtable);
        break;
      case 12:
        mostrarGrafo(graph);
        break;
      default:
        std::cout << "Opción inválida. Intente nuevamente." << std::endl;
        break;
    }

    std::cout << std::endl;
  }

  return 0;
}