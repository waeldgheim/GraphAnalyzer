#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include<stack>
#include <queue>
using namespace std;

struct Node {
    string label;
    unordered_map<Node*, int> neighbors;
    int indegree = 0;
};

struct Graph {
    bool isDirected;
    unordered_map<string, Node*> nodes;


    Graph(bool isDirected) : isDirected(isDirected) {}

    void addNode(string label) {
        // Check if the node already exists
        if (nodes.count(label)) {
            cout << "Node " << label << " already exists." << endl;
            return;
        }

        // Create the new node
        Node* newNode = new Node;
        newNode->label = label;

        nodes[label] = newNode;
    }

    void addEdge(string fromLabel, string toLabel, float weight) {
        // Get pointers to the from and to nodes
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];
        toNode->indegree++;
        // Add the edge from the from node to the to node
        fromNode->neighbors[toNode] = weight;

        // If the graph is undirected, add an edge from the to node to the from node
        if (!isDirected) {
            toNode->neighbors[fromNode] = weight;
            fromNode->indegree++;
        }
    }

    void removeNode(string label) {
        // Check if the node doesn't exist
        if (!nodes.count(label)) {
            cout << "Node " << label << " doesn't exist." << endl;
            return;
        }

        Node* node = nodes[label];

        // Remove the node from the graph
        nodes.erase(label);

        // Remove all edges to and from the node
        for (auto neighbor : node->neighbors) {
            Node* toNode = neighbor.first;
            toNode->neighbors.erase(node);
        }
        node->neighbors.clear();

        // Delete the node
        delete node;
    }
    
    void removeEdge(string fromLabel, string toLabel) {
        // Get pointers to the from and to nodes
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];

        // Remove the edge from the from node to the to node
        fromNode->neighbors.erase(toNode);
        toNode->indegree--;

        // If the graph is undirected, also remove the edge from the to node to the from node
        if (!isDirected) {
            toNode->neighbors.erase(fromNode);
            fromNode->indegree--;
        }
    }
    void adjustWeight(string fromLabel, string toLabel, float newWeight) {
        // Get pointers to the from and to nodes
        Node* fromNode = nodes[fromLabel];
        Node* toNode = nodes[toLabel];

        // Update the weight of the edge from the from node to the to node
        fromNode->neighbors[toNode] = newWeight;

        // If the graph is undirected, also update the weight of the edge from the to node to the from node
        if (!isDirected) {
            toNode->neighbors[fromNode] = newWeight;
        }
    }

    void DFS(string s ) {
        unordered_map<string, bool> visited;

        stack<string> stack;
        stack.push(s);

        // Perform DFS traversal
        while (!stack.empty()) {
            string v = stack.top();
            stack.pop();

            if (!visited[v]) {
                visited[v] = true;
                cout << v << " ";

                for (auto it : nodes[v]->neighbors){
                    stack.push(it.first->label);
                }
            }
        }
    }

    bool isCyclic_util(unordered_map<string, Node*> nodes, unordered_map<string, bool> visited, string curr) {
        if (visited[curr] == true) { return true; }

        visited[curr] = true;
        bool f = false;
        for (auto it : nodes[curr]->neighbors) {
            f = isCyclic_util(nodes, visited, it.first->label);
            if (f) { return true; }
        }
        return false;
    }

    bool isCyclic(){
        unordered_map<string, bool> visited;
        for (auto i : nodes) { visited[i.first] = false; }

        bool f = false;
        for (auto j : nodes) {
            visited[j.first] = true;
            for (auto k : j.second->neighbors) {
                f = isCyclic_util(nodes, visited, k.first->label);
                if (f) { return true; }
            }
            visited[j.first] = false;
        }
        return false;
    }


    void bellman_ford(string source,string dest) {

        // Initialize distances and parent for each vertex
        unordered_map<string, float> distances;
        unordered_map<string, string> parent;
        for (auto vertex : nodes) {
            distances[vertex.first] = 984958;
            parent[vertex.first] = "";
        }
        distances[source] = 0;

        // Relax edges |V|-1 times
        int V = nodes.size();
        for (int i = 0; i < V - 1; i++) {
            for (auto u : nodes) {
                for (auto v : u.second->neighbors) {
                    if (distances[u.first] + v.second < distances[v.first->label]) {
                        distances[v.first->label] = distances[u.first] + v.second;
                        parent[v.first->label] = u.first;
                    }
                }
            }
        }

        bool hasNegativeCycle = false;
        for (auto it: nodes) {
            for (auto j : it.second->neighbors) {
                if (distances[it.first] + j.second< distances[j.first->label]) {
                    hasNegativeCycle = true;
                    break;
                }
            }
        }

        if (hasNegativeCycle) {
            std::cout << "The graph contains a negative cycle." << std::endl;
            return;
        }

        vector<string> path;
        string current = dest;
        while (current != source) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);
        
        cout << "The shortest path between " << source << " and " << dest << " is ";
        for (int i = path.size()-1; i>=0; i--) {
            cout << path[i] << " ";
        }
    }

    void Dijkstra(string source, string dest){
        
        unordered_map<string, float> distances;
        unordered_map<string, string> parent;
        for (auto vertex : nodes) {
            distances[vertex.first] = 984958;
            parent[vertex.first] = "";
        }
        distances[source] = 0;

        // Use a min-priority queue to select the vertex with the smallest distance
        queue<string> q;
        q.push(source);

        while (!q.empty()) {
            string u = q.front();
            q.pop();

            for (auto v : nodes[u]->neighbors) {            
                if (distances[u] + v.second < distances[v.first->label]) {
                    distances[v.first->label] = distances[u] + v.second;
                    parent[v.first->label] = u;
                    q.push(v.first->label);
                }
            }
        }
        vector<string> path;
        string current = dest;
        while (current != source) {
            path.push_back(current);
            current = parent[current];
        }
        path.push_back(source);

        cout << "The shortest path between " << source << " and " << dest << " is ";
        for (int i = path.size() - 1; i >= 0; i--) {
            cout << path[i] << " ";
        }

    }

    void topologicalSort() {
            vector<string> result; // vector to store the result
            unordered_map<string, bool> visited2;
            stack<string> stack; // stack to store the nodes in topological order

            // Start with the nodes with no incoming edges
            for (auto it: nodes) {
                if (it.second->indegree == 0) {
                    stack.push(it.first);
                }
            }

            while (!stack.empty()) {
                string node = stack.top();
                stack.pop();

                if (!visited2[node]) {
                    // Mark the node as visited and add it to the result
                    visited2[node] = true;
                    result.push_back(node);

                    // Add the neighbors of the node to the stack
                    for (auto j: nodes[node]->neighbors) {
                        j.first->indegree = j.first->indegree - 1;
                        if (j.first->indegree == 0) {
                            stack.push(j.first->label);
                        }
                    }
                }
            }

        for (auto i : result) {
            cout << i << " ";
        }
        cout << endl;
    }
};

int main() {
    bool negWeight = false;
    cout << "Is the graph directed or undirected? (Enter 1 for directed, 0 for undirected): ";
    int isDirected;
    cin >> isDirected;

    Graph graph(isDirected);

   
    while (true) {
        cout << "Enter a command \n0 Exit\n1 Add a node\n2 Remove a node\n3 Add an edge\n4 Adjust the weight of an edge\n5 Remove an edge\n6 DFS Traversal\n7 Detect Cycle\n8 Topological Sort\n9 Find Shortest Path\n>> ";
        int command;
        cin >> command;

        cout << endl;

        if (command == 0) {     //exit code
            break;
        }
        else if (command == 1) {        //add node

            cout << "Enter a label for the node to add: ";
            string label;
            cin >> label;
            graph.addNode(label);
        }
        else if (command == 2) {        //remove node

            cout << "Enter the label of the node to remove: ";
            string label;
            cin >> label;
            graph.removeNode(label);
            cout << endl;
        }

        if (command == 3) {         //add edge

            cout << "Enter the labels of the two nodes to add an edge between: ";
            string label1, label2;
            cin >> label1 >> label2;

            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                cout << "Is the edge weighted? (Enter 1 for yes, 0 for no): ";
                int isWeighted;
                cin >> isWeighted;

                if (isWeighted) {
                    cout << "Enter the weight of the edge: ";
                    double weight;
                    cin >> weight;
                    if (weight < 0) { negWeight = true; }
                    graph.addEdge(label1, label2, weight);
                }
                else {
                    graph.addEdge(label1, label2, 0);
                }
            }
            else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        }
        if (command == 4) {         //Adjust the weight of an edge

            cout << "Enter the labels of the two nodes: ";
            string label1, label2;
            cin >> label1 >> label2;

            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                graph.removeEdge(label1, label2);
                cout << "Enter the new weight of the edge: ";
                float weight;
                cin >> weight;
                if (weight < 0) { negWeight = true; }
                graph.adjustWeight(label1, label2, weight);
            }
            else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
             
        }

        if (command == 5) {         //remove an edge

            cout << "Enter the labels of the two nodes to remove the edge between: ";
            string label1, label2;
            cin >> label1 >> label2;

            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                graph.removeEdge(label1, label2);
            }
            else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
           
        }

        if (command == 6) {     //DFS

            cout << "Enter the label of the node: ";
            string label1;
            cin >> label1;
            if (graph.nodes.count(label1)) {
                graph.DFS(label1);
            }
            else {
                cout << "The node is not found";
            }
            cout << endl;
        }

        if (command == 7) {         //detect cycle

            bool t = graph.isCyclic();
            if (t) { cout << "Graph is cyclic."; }
            else { cout << "Graph is not cyclic."; }
            cout << endl;
           
        }

        if (command == 8) {         //topological sort

            bool t;
            for (auto it : graph.nodes) {
                t = graph.isCyclic();
                if (graph.isCyclic() || !isDirected) { cout << "Can't perform Topological Sort on undirected or cyclic graph"<<endl; }
                break;
            }
            if (!t) {
                graph.topologicalSort();
            }
            cout << endl;
        }

        if (command == 9) {         //shortest path

            cout << "Enter the labels of the two nodes: ";
            string label1, label2;
            cin >> label1 >> label2;

            if (graph.nodes.count(label1) && graph.nodes.count(label2)) {
                if (negWeight) {
                    graph.bellman_ford(label1, label2);
                }
                else { graph.Dijkstra(label1, label2); }
            }

            else {
                cout << "One or both nodes are not found";
            }
            cout << endl;
        }
    }
    if (false) {
        cout << "Invalid command." << endl;
    }
    // Printing the graph
    for (auto nodePair : graph.nodes) {
        Node* node = nodePair.second;

        cout << "Node " << node->label << " has neighbors: ";

        for (auto neighborPair : node->neighbors) {
            Node* neighbor = neighborPair.first;
            int weight = neighborPair.second;

            cout << neighbor->label << " (" << weight << ") ";
        }

        cout << endl;

    }

    return 0;
}