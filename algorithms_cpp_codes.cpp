//astar algorithm

#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <bits/stdc++.h>

using namespace std;

// define the graph data structure
struct Edge {
    int to, cost;
    Edge(int _to, int _cost) : to(_to), cost(_cost) {}
};

typedef vector<vector<Edge>> Graph;

// define the state data structure
struct State {
    int node, f;
    State(int _node, int _f) : node(_node), f(_f) {}
    bool operator < (const State& rhs) const {
        return f > rhs.f;
    }
};

// define the heuristic function (in this case, just use the distance)
int heuristic(int a, int b) {
    return abs(a - b);
}

// implementation of A* algorithm
vector<int> astar(const Graph& graph, int start, int goal) {
    priority_queue<State> pq;
    pq.emplace(start, 0);

    unordered_map<int, int> g;
    g[start] = 0;

    unordered_map<int, int> parent;

    while (!pq.empty()) {
        auto current = pq.top().node;
        pq.pop();

        if (current == goal) {
            break;
        }

        for (auto& edge : graph[current]) {
            int next = edge.to;
            int cost = edge.cost;
            int new_g = g[current] + cost;

            if (g.find(next) == g.end() || new_g < g[next]) {
                g[next] = new_g;
                int f = new_g + heuristic(next, goal);
                pq.emplace(next, f);
                parent[next] = current;
            }
        }
    }

    vector<int> path;
    for (int node = goal; node != start; node = parent[node]) {
        path.push_back(node);
    }
    path.push_back(start);
    reverse(path.begin(), path.end());

    return path;
}

int main() {
    int node ;
    cout<<"Enter the number of Nodes: ";
    cin >> node;

    Graph graph(node + 1);
    int edges;
    cout<<"Enter the number of Edges: ";
    cin>> edges;
    for (int i = 0; i < edges; i++) {
        int from,to,cost;
        cout<<"Enter the path :\n";
        cin>>from >> to >> cost;
        graph[from].push_back(Edge(to,cost));
    } 

    int start, goal;
    cout<<"Enter the starting node and destination node :\n";
    cin>> start >> goal;
    vector<int> path = astar(graph, 0, 4);

    for (auto& node : path) {
        cout << node << " ";
    }
    cout << endl;

    return 0;
}




//tspbfs

#include <bits/stdc++.h>
using namespace std;

const int N = 20;
int n, graph[N][N], visited[1 << N][N], ans = INT_MAX;

struct node
{
  int mask, cur, cost;
};

void bfs()
{
  queue<node> q;
  q.push({1, 0, 0});
  visited[1][0] = 1;
  while (!q.empty())
  {
    node nd = q.front();
    q.pop();
    if (nd.mask == (1 << n) - 1 && graph[nd.cur][0])
    {
      ans = min(ans, nd.cost + graph[nd.cur][0]);
    }
    for (int i = 0; i < n; i++)
    {
      if (!visited[nd.mask | (1 << i)][i] && graph[nd.cur][i])
      {
        visited[nd.mask | (1 << i)][i] = 1;
        q.push({nd.mask | (1 << i), i, nd.cost + graph[nd.cur][i]});
      }
    }
  }
}

int main()
{
  cout << "Enter the number of nodes: ";
  cin >> n;
  cout << "Enter the adjacency matrix: \n";
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      cin >> graph[i][j];
    }
  }
  bfs();
  if (ans == INT_MAX)
  {
    cout << "No Hamiltonian cycle exists.\n";
  }
  else
  {
    cout << "Minimum cost of Hamiltonian cycle: " << ans << "\n";
  }
  return 0;
}





//tspdfs

#include <bits/stdc++.h>
using namespace std;

vector<int> path;
vector<int> min_path;
int min_tsp = INT_MAX - 1;
void dfs(vector<vector<int>> &adj, int node, int cnt, vector<bool> visited, int cost)
{
  if (cnt == adj.size() && adj[node][0] > 0)
  {
    if (min_tsp > cost + adj[node][0])
    {
      min_tsp = cost + adj[node][0];
      min_path = path;
    }
    return;
  }
  for (int i = 0; i < adj.size(); i++)
  {
    if (adj[node][i] <= 0 || visited[i])
      continue;
    visited[i] = true;
    path.push_back(i);
    dfs(adj, i, cnt + 1, visited, cost + adj[node][i]);
    path.pop_back();
    visited[i] = false;
  }
}
int main()
{
  cout << "\n----TRAVELLING SALESMAN PROBLEM USING DEPTH FIRST SEARCH----\n";
  int n = 7;
  vector<vector<int>> adj(n, vector<int>(n, -1));
  cout << "\nEnter the adjacency matrix\n";
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
      cin >> adj[i][j];
  }
  cout << "\n";
  vector<bool> visited(n, false);
  visited[0] = true;
  path.push_back(0);
  dfs(adj, 0, 1, visited, 0);
  cout << "Minimum cost path: ";
  for (int i = 0; i < min_path.size(); i++)
  {
    cout << min_path[i] << " -> ";
  }
  cout << 0 << endl;
  cout << "\n";
  cout << "Minimum Cost: " << min_tsp << endl;
  cout << "\n";
}