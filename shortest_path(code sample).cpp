#include "shortestpath.h"
#include "Vertex.h"
#include "graph.h"
#include "edge.h"
#include <iostream>
#include <queue>
#include <string>
#include <vector>
#include <limits.h>
#include <float.h>
#include <tuple>

ShortestPath::ShortestPath(Vertex _source, Vertex _destination) {
    source = _source;
    destination = _destination;
}

std::vector<Edge> ShortestPath::Dijkstra(Graph G) {
    // vector containing all the pathes (Edges) in the shortest distance
    std::vector<Edge> shortestPaths;

    // the minHeap used in Dijkstra's algorithm
    std::priority_queue<pair<int, long double>, std::vector<pair<int, long double>>, TypeComparator> pq;
    
    for (Vertex V : G.getVertices()) {
        if (source.airportID == V.airportID)
            mm[V.airportID] = {0, Vertex(), false};
        else
            mm[V.airportID] = {DBL_MAX, Vertex(), false};
    }

    pq.push({source.airportID, get<0>(mm[source.airportID])});

    // Dijkstra's algorithm
    while (!pq.empty()) {
        pair<int, long double> currentP = pq.top();
        Vertex currentV = G.m[currentP.first];
        pq.pop();

        // if current Vertex is the destination
        // then trace back from destination to source to get all pathes in the shortest distance
        if (currentV == destination) {
            shortestDistance = get<0>(mm[currentV.airportID]);
            std::cout << "Shortest Distance: " << shortestDistance << " KM" << std::endl;
            Vertex pre = get<1>(mm[currentV.airportID]);
            Edge path = G.getEdge(pre, currentV);
            shortestPaths.push_back(path);
            while (pre != source) {
                Vertex curr = pre;
                pre = get<1>(mm[pre.airportID]);
                path = G.getEdge(pre, curr);
                shortestPaths.push_back(path);
            }

            // print out information about the shortest path
            for (int i = shortestPaths.size() - 1; i >= 0; i--)
                std::cout<< "path" << shortestPaths.size() - i << ": " << shortestPaths[i].source.nameOfAirport << " -> " 
                    << shortestPaths[i].destination.nameOfAirport << " " << shortestPaths[i].getWeight() << " KM" << std::endl;

            // reverse the shortestPaths array to make the first item the source path and the last item the destination path
            std::reverse(shortestPaths.begin(), shortestPaths.end());
            mm.clear();
            std::cout<<std::endl;
            return shortestPaths;
        }

        // search adjacent Vertices if current Vertex is not the destination
        for (Vertex W : G.getAdjacent(currentV)) {
           long double distance = G.getEdgeWeight(currentV, W) + get<0>(mm[currentV.airportID]);
            if (distance < get<0>(mm[W.airportID])) {
                get<0>(mm[W.airportID]) = distance;
                get<1>(mm[W.airportID]) = currentV;
                pq.push({W.airportID, get<0>(mm[W.airportID])});
            }
        }
    }

    // if destination is not in the graph, output an array with an empty Edge
    std::cout<< "No path between " << source.nameOfAirport << " and " << destination.nameOfAirport << std::endl;
    shortestPaths.push_back(Edge());
    mm.clear();
    return shortestPaths;
}

long double ShortestPath::getShortestDistance() {
    return shortestDistance;
}
