#define GRAPH_H
#define MESSAGE_H
#define NODE_H
#define PATH_H
#define PACKET_H
#define NETWORK_H
#define STRINGSPLITTER_H

#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <unordered_map>
#include <stack>
#include <iomanip>
#include <vector>
#include <limits>
#include <algorithm>

using namespace std;

class Node
{
private:
    int _id;
    int _load_factor;
    unordered_map<Node *, int> _edges;
public:
    Node(int id);
    int getId() const;
    int getLoadFactor() const;
    int getWeight(Node *node);
    void increaseLoadFactor();
    void decreaseLoadFactor();
    void addEdge(Node *node, int weight);
    unordered_map<Node *, int> getEdges();
};
Node::Node(int id)
{
    _id = id;
    _load_factor = 1;
}
int Node::getId() const
{
    return _id;
}
int Node::getLoadFactor() const
{
    return _load_factor;
}
int Node::getWeight(Node *node)
{
    return _edges[node];
}
void Node::increaseLoadFactor()
{
    _load_factor++;
}
void Node::decreaseLoadFactor()
{
    _load_factor--;
}
void Node::addEdge(Node *node, int weight)
{
    _edges[node] = weight;
}
unordered_map<Node *, int> Node::getEdges()
{
    return _edges;
}

class Path
{
private:
    stack<Node *> _node_path;
    int _distance_traveled;
public:
    Path();
    Path(Node *node);
    Path(stack<Node *> previous_path, int distance);
    void addNodeToPath(Node *start_node, Node *end_node);
    Node *getTopNode();
    void dequeuePath();
    bool isEmpty() const;
    int getDistance() const;
    stack<Node *> getPath() const;
    stack<Node *> getPathQueue();
};

struct PathCompare
{
    bool operator()(const Path *path1, const Path *path2) const
    {
        return path1->getDistance() > path2->getDistance();
    }
};
Path::Path()
{
    _distance_traveled = 0;
}
Path::Path(Node *node)
{
    _node_path.push(node);
    _distance_traveled = 0;
}
Path::Path(stack<Node *> previous_path, int distance)
{
    _node_path = previous_path;
    _distance_traveled = distance;
}
void Path::addNodeToPath(Node *start_node, Node *end_node)
{
    _node_path.push(end_node);
    _distance_traveled += start_node->getWeight(end_node) * end_node->getLoadFactor();
}
Node* Path::getTopNode()
{
    return _node_path.top();
}
void Path::dequeuePath()
{
    _node_path.pop();
}
bool Path::isEmpty() const
{
    return _node_path.empty();
}
int Path::getDistance() const
{
    return _distance_traveled;
}
stack<Node *> Path::getPath() const
{
    return _node_path;
}
stack<Node *> Path::getPathQueue()
{
    stack<Node *> queue;
    while (_node_path.empty() == false)
    {
        Node *node = _node_path.top();
        queue.push(node);
        _node_path.pop();
    }
    return queue;
}

class Packet
{
private:
    char _value;
    int _order;
    int _current_wait;
    int _arrival_time;
    Node *_destination;
    Node *_previous_location;
    Node *_next_location;
    queue<Node *> _visited_nodes;
public:
    Packet(char value, int order, Node *destination, Node *start_location);
    char getValue() const;
    int getOrder() const;
    int getCurrentWait() const;
    Node *getDestination() const;
    Node *getPreviousLocation() const;
    Node *getNextLocation() const;
    int getArrivalTime() const;
    void setPreviousLocation(Node *node);
    void setNextLocation(Node *node);
    void setCurrentWait(int wait);
    void setStartTime(int start);
    bool tick();
    void addVisitedNode(Node *node);
    int getTransmissionTime() {return _order;};
    int getPriority() {return _order;};
    queue<Node *> getVisitedNodes();
};
Packet::Packet(char value, int order, Node *destination, Node *start_location)
{
    _value = value;
    _order = order;
    _current_wait = -1; // Means not in the network yet
    _arrival_time = 0;
    _destination = destination;
    _previous_location = start_location;
    _next_location = nullptr;
}
char Packet::getValue() const
{
    return _value;
}
int Packet::getOrder() const
{
    return _order;
}
int Packet::getCurrentWait() const
{
    return _current_wait;
}
Node* Packet::getDestination() const
{
    return _destination;
}
Node* Packet::getPreviousLocation() const
{
    return _previous_location;
}
Node* Packet::getNextLocation() const
{
    return _next_location;
}
int Packet::getArrivalTime() const
{
    return _arrival_time;
}
void Packet::setPreviousLocation(Node *node)
{
    _previous_location = node;
}
void Packet::setNextLocation(Node *node)
{
    _next_location = node;
}
void Packet::setCurrentWait(int wait)
{
    _current_wait = wait;
    _arrival_time += wait;
}
void Packet::setStartTime(int start)
{
    _arrival_time = start;
}
bool Packet::tick()
{
    if (_current_wait > 0)
    {
        _current_wait--;
        return false;
    }
    
    else if (_current_wait == 0)
    {
        return true;
    }
    
    else
    {
        return true;
    }
}
void Packet::addVisitedNode(Node *node)
{
    _visited_nodes.push(node);
}
queue<Node *> Packet::getVisitedNodes()
{
    return _visited_nodes;
}
class Graph
{
private:
    unordered_map<int, Node *> _nodes;
public:
    void addNode(int node_id);
    void addEdge(int start_id, int end_id, int weight);
    Node *getNode(int node_id);
    Path *computeShortestPath(int start_id, int end_id);
};
void Graph::addNode(int node_id)
{
    _nodes[node_id] = new Node(node_id);
}
void Graph::addEdge(int start_id, int end_id, int weight)
{
    _nodes[start_id]->addEdge(_nodes[end_id], weight);
}
Node* Graph::getNode(int node_id)
{
    return _nodes[node_id];
}
Path* Graph::computeShortestPath(int start_id, int end_id)
{
    unordered_map<int, Path *> finished;
    priority_queue<Path *, vector<Path *>, PathCompare> working_nodes;
    Node *current_node = _nodes[start_id];
    working_nodes.push(new Path(current_node));
    while (working_nodes.empty() == false)
    {
        Path *path = working_nodes.top();
        working_nodes.pop();
        current_node = path->getTopNode();
        unordered_map<Node *, int> edges = current_node->getEdges();
        for (unordered_map<Node *, int>::iterator it = edges.begin(); it != edges.end(); it++)
        {
            unordered_map<int, Path*>::const_iterator cit = finished.find(it->first->getId());
            //if (finished[it->first->getId()] == nullptr)
            if (cit == finished.end())
            {
                Path *new_path = new Path(path->getPath(), path->getDistance());
                new_path->addNodeToPath(new_path->getTopNode(), it->first);
                working_nodes.push(new_path);
            }
        }
        
        if (finished[current_node->getId()] == nullptr)
        {
            finished[current_node->getId()] = path;
        }
            
        if (path->getDistance() < finished[current_node->getId()]->getDistance())
        {
            finished[current_node->getId()] = path;
        }
            
        if (finished.size() == _nodes.size())
        {
            break;
        }
    }
        
    return finished[end_id];
}
class Message
{
private:
    queue<Packet *> _packets;
    Node *_starting_node;
    Node *_ending_node;
    int _size;
public:
    Message();
    Message(string message, Node *starting_node, Node *ending_node);
    bool isEmpty() const;
    Packet *getNextPacket();
    int getSize() const;
};
Message::Message()
{
    _starting_node = nullptr;
    _ending_node = nullptr;
    _size = 0;
}
Message::Message(string message, Node *starting_node, Node *ending_node)
{
    _starting_node = starting_node;
    _ending_node = ending_node;
    for (int index = 0; index < message.length(); index++)
    {
        _packets.push(new Packet(message[index], index, ending_node, starting_node));
    }
    _size = _packets.size();
}
bool Message::isEmpty() const
{
    return _packets.empty();
}
Packet* Message::getNextPacket()
{
    Packet *packet = _packets.front();
    _packets.pop();
    return packet;
}
int Message::getSize() const
{
    return _size;
}
class StringSplitter
{
public:
    
    static string * split(string text, string delimiter, int &items_found)
    {
        //vectors are dynamically expanding arrays
        vector<string> pieces;
        
        //find the first delimiter
        int location = text.find(delimiter);
        
        //we are starting at the beginning of our string
        int start = 0;
        
        //go until we have no more delimiters
        while(location != string::npos)
        {
            //add the current piece to our list of pieces
            string piece = text.substr(start, location - start);
            pieces.push_back(piece);
            
            //update our index markers for the next round
            start = location + 1;
            location = text.find(delimiter, start);
        }
        
        //at the end of our loop, we're going to have one trailing piece to take care of.
        //handle that now.
        string piece = text.substr(start, location - start);
        pieces.push_back(piece);
        
        //convert from vector into an array of strings
        int size = pieces.size();
        string *pieces_str = new string[size];
        for(int i = 0; i < size; i++)
        {
            pieces_str[i] = pieces.at(i);
        }
        items_found = size;
        return pieces_str;			
    }
};
vector<Packet*> schedulePacketsDP(vector<Packet*>& packets) {
    // Sort packets by finish time (current implementation time + transmission time)
    sort(packets.begin(), packets.end(), [](Packet* a, Packet* b) {
        return (a->getArrivalTime() + a->getTransmissionTime()) < 
               (b->getArrivalTime() + b->getTransmissionTime());
    });

    int n = packets.size();
    vector<int> dp(n + 1, 0);
    vector<int> prev(n + 1, -1);

    // Precompute closest non-conflicting packet
    vector<int> lastNonConflict(n, -1);
    for (int i = 1; i < n; i++) {
        for (int j = i-1; j >= 0; j--) {
            if (packets[j]->getArrivalTime() + packets[j]->getTransmissionTime() <= 
                packets[i]->getArrivalTime()) {
                lastNonConflict[i] = j;
                break;
            }
        }
    }

    // DP table construction
    for (int i = 1; i <= n; i++) {
        int inclProfit = packets[i-1]->getPriority();
        if (lastNonConflict[i-1] != -1)
            inclProfit += dp[lastNonConflict[i-1] + 1];
        
        if (inclProfit > dp[i-1]) {
            dp[i] = inclProfit;
            prev[i] = lastNonConflict[i-1] + 1;
        } else {
            dp[i] = dp[i-1];
        }
    }

    // Backtrack to find selected packets
    vector<Packet*> result;
    int i = n;
    while (i > 0) {
        if (dp[i] != dp[i-1]) {
            result.push_back(packets[i-1]);
            i = prev[i];
        } else {
            i--;
        }
    }
    reverse(result.begin(), result.end());
    return result;
}
int fordFulkerson(const vector<vector<int>>& capacity,int source,int sink){
    int n = capacity.size();
    // residual capacities initialized to original capacities
    vector<vector<int>> residual = capacity;
    vector<int> parent(n);
    int maxFlow = 0;

    // Helper: find an augmenting path via BFS, fill parent[], return flow
    auto bfs = [&]() -> int {
    fill(parent.begin(), parent.end(), -1);
    parent[source] = source;
    queue<pair<int,int>> q;
    q.push({source, numeric_limits<int>::max()});

    while (!q.empty()) {
    auto it = q.front();
    int u = it.first;
    int flow = it.second;
    q.pop();

    for (int v = 0; v < n; ++v) {
    if (parent[v] == -1 && residual[u][v] > 0) {
        // possible to send flow
        parent[v] = u;
        int new_flow = min(flow, residual[u][v]);
        if (v == sink) {
            return new_flow;
        }
        q.push({v, new_flow});
    }
    }
    }
    return 0; // no augmenting path found
    };

    // Repeatedly find augmenting paths and update residual graph
    while (int flow = bfs()) {
    maxFlow += flow;
    int v = sink;
    // walk back from sink to source via parent[]
    while (v != source) {
    int u = parent[v];
    residual[u][v] -= flow;      // reduce forward capacity
    residual[v][u] += flow;      // increase reverse capacity
    v = u;
    }
    }

    return maxFlow;
}
class Network
{
private:
    Graph _graph;
    int _tick_count;
    Message _message;
    unordered_map<int, Packet *> _packets;
    unordered_map<int, Packet *> _delivered;
public:
    Network();
    void addNode(int node_id);
    void addEdge(int start_id, int end_id, int weight);
    void addMessage(string message, int start_id, int end_id);
    Path *computeShortestPath(int start_id, int end_id);
    void createGraph(ifstream &input_file);
    void tick();
    bool isRunning() const;
    void displayResults();
};
Network::Network()
{
    _tick_count = 0;
}
void Network::addNode(int node_id)
{
    _graph.addNode(node_id);
}
void Network::addEdge(int start_id, int end_id, int weight)
{
    _graph.addEdge(start_id, end_id, weight);
}
void Network::addMessage(string message, int start_id, int end_id)
{
    _message = Message(message, _graph.getNode(start_id), _graph.getNode(end_id));
}
Path* Network::computeShortestPath(int start_id, int end_id)
{
    return _graph.computeShortestPath(start_id, end_id);
}
void Network::createGraph(ifstream &input_file)
{
    string line;
    int items_found = 0;
    while (input_file.good() == true)
    {
        getline(input_file, line);
        string *pieces = StringSplitter::split(line, " ", items_found);
        if (items_found == 1)
        {
            addNode(stoi(pieces[0]));
        }
        
        else if (items_found == 3)
        {
            addEdge(stoi(pieces[0]), stoi(pieces[1]), stoi(pieces[2]));
        }
        
        else
        {
            exit(0);
        }
    }
}
void Network::tick()
{
    _tick_count++;
    if (_message.isEmpty() == false)
    {
        Packet *packet = _message.getNextPacket();
        Path *path = computeShortestPath(packet->getPreviousLocation()->getId(), packet->getDestination()->getId());
        stack<Node *> path_stack = path->getPathQueue();
        Node *previous = path_stack.top();
        path_stack.pop(); 
        Node *next = path_stack.top();
        path_stack.pop();
        packet->setPreviousLocation(previous);
        packet->setNextLocation(next);
        packet->setStartTime(_tick_count);
        packet->setCurrentWait(_graph.getNode(previous->getId())->getWeight(next) * next->getLoadFactor());
        packet->addVisitedNode(previous);
        previous->increaseLoadFactor();
        next->increaseLoadFactor();
        _packets[packet->getOrder()] = packet;
        cout << "Sending packet " << packet->getValue() << " to vertex " << packet->getNextLocation()->getId() <<
        " with a wait of " << packet->getCurrentWait() << " at time " << _tick_count << endl;
        
    }
    
    for (unordered_map<int, Packet *>::iterator it = _packets.begin(); it != _packets.end(); it++)
    {
        bool finished = it->second->tick();
        if (finished == true)
        {
            if (it->second->getDestination() != it->second->getNextLocation())
            {
                it->second->getPreviousLocation()->decreaseLoadFactor();
                it->second->getNextLocation()->decreaseLoadFactor();
                Path *path = computeShortestPath(it->second->getNextLocation()->getId(), it->second->getDestination()->getId());
                stack<Node *> path_stack = path->getPathQueue();
                it->second->addVisitedNode(it->second->getNextLocation());
                Node *previous = path_stack.top();
                path_stack.pop();
                Node *next = path_stack.top();
                path_stack.pop();
                it->second->setPreviousLocation(previous);
                it->second->setNextLocation(next);
                it->second->setCurrentWait(_graph.getNode(previous->getId())->getWeight(next) * next->getLoadFactor());
                previous->increaseLoadFactor();
                next->increaseLoadFactor();
                cout << "Sending packet " << it->second->getValue() << " to vertex " << it->second->getNextLocation()->getId() <<
                " with a wait of " << it->second->getCurrentWait() << " at time " << _tick_count << endl;
            }
            
            else if (_delivered[it->second->getOrder()] == nullptr)
            {
                it->second->addVisitedNode(it->second->getNextLocation());
                it->second->getPreviousLocation()->decreaseLoadFactor();
                it->second->getNextLocation()->decreaseLoadFactor();
                _delivered[it->second->getOrder()] = it->second;
            }
        }
    }
}
bool Network::isRunning() const
{
    if (_delivered.size() == _message.getSize())    // terminates when message size and delivered map size becomes equal
    {
        return false;
    }
    else
    {
        return true;
    }
}
void Network::displayResults()
{
    cout << "Packet - Arrival Time - Route" << endl;
    for (int index = 0; index < _message.getSize(); index++)
    {
        cout << _delivered[index]->getValue() << setw(10) << _delivered[index]->getArrivalTime() << setw(14);
        queue<Node *> visited = _delivered[index]->getVisitedNodes();
        while (visited.empty() == false)
        {
            cout << visited.front()->getId();
            if (visited.size() > 1)
            {
                cout << ", ";
                visited.pop();
            }
            else
            {
                cout << endl;
                visited.pop();
            }
        }
    }
}
int main()
{
	Network network;
	string line;     // for storing name of graph file
	ifstream input_file;     
	int start_id = 0, end_id = 0;

	cout << "Enter graph file: ";
	getline(cin, line);
	input_file.open(line);
	network.createGraph(input_file);
	cout << "Enter a starting vertex: ";
	getline(cin, line);
	start_id = stoi(line);
	cout << "Enter a destination vertex: ";
	getline(cin, line);
	end_id = stoi(line);
	cout << "Enter a message to transmit: ";
	getline(cin, line);
	network.addMessage(line, start_id, end_id);
	while (network.isRunning() == true)
	{
		network.tick();
	}
	network.displayResults();
    return 0;
}