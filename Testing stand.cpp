#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <chrono>   // Для замеров времени
#include <random>   // Для генерации случайных графов
#include <iomanip>  // Для форматирования таблицы
#include <algorithm>

using namespace std;

const double INF = numeric_limits<double>::infinity();

struct Point {
    double x, y;
};

struct Edge {
    int to;
    double weight;
};

// --- 1. РЕАЛИЗАЦИЯ АЛГОРИТМОВ ---

// Эвристика: Евклидово расстояние
double Heuristic(int u, int v, const vector<Point>& coords) {
    double dx = coords[u].x - coords[v].x;
    double dy = coords[u].y - coords[v].y;
    return sqrt(dx * dx + dy * dy);
}

// Алгоритм A* (A-Star)
bool Run_A_Star(int start, int goal, int N, const vector<vector<Edge>>& adj, const vector<Point>& coords) {
    vector<double> g_score(N, INF);
    vector<double> f_score(N, INF);
    priority_queue<pair<double, int>> open_set;

    g_score[start] = 0.0;
    f_score[start] = Heuristic(start, goal, coords);
    open_set.push({ -f_score[start], start });

    while (!open_set.empty()) {
        int current = open_set.top().second;
        double current_f = -open_set.top().first;
        open_set.pop();

        if (current == goal) return true;

        if (current_f > f_score[current]) continue;

        for (const auto& edge : adj[current]) {
            int neighbor = edge.to;
            double tentative_g = g_score[current] + edge.weight;

            if (tentative_g < g_score[neighbor]) {
                g_score[neighbor] = tentative_g;
                f_score[neighbor] = tentative_g + Heuristic(neighbor, goal, coords);
                open_set.push({ -f_score[neighbor], neighbor });
            }
        }
    }
    return false;
}

// Алгоритм Флойда-Уоршелла
void Run_Floyd_Warshall(int N, const vector<vector<double>>& matrix_input) {
    vector<vector<double>> S = matrix_input;

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                if (S[i][k] != INF && S[k][j] != INF) {
                    if (S[i][k] + S[k][j] < S[i][j]) {
                        S[i][j] = S[i][k] + S[k][j];
                    }
                }
            }
        }
    }
}

// --- 2. ГЕНЕРАЦИЯ ДАННЫХ ---

struct GraphData {
    int N;
    vector<Point> coords;
    vector<vector<Edge>> adjList;
    vector<vector<double>> adjMat;
};

GraphData GenerateGraph(int N, double density) {
    GraphData data;
    data.N = N;
    data.coords.resize(N);
    data.adjList.resize(N);
    data.adjMat.assign(N, vector<double>(N, INF));

    random_device rd;
    mt19937 gen(rd());
    uniform_real_distribution<> coord_dist(0.0, 1000.0);
    uniform_real_distribution<> prob_dist(0.0, 1.0);

    for (int i = 0; i < N; ++i) {
        data.coords[i] = { coord_dist(gen), coord_dist(gen) };
        data.adjMat[i][i] = 0.0;
    }

    for (int i = 0; i < N; ++i) {
        for (int j = i + 1; j < N; ++j) {
            if (prob_dist(gen) < density) {
                double dist = Heuristic(i, j, data.coords);
                data.adjList[i].push_back({ j, dist });
                data.adjList[j].push_back({ i, dist });
                data.adjMat[i][j] = dist;
                data.adjMat[j][i] = dist;
            }
        }
    }
    return data;
}

// --- 3. ОСНОВНОЙ ЦИКЛ ТЕСТИРОВАНИЯ ---

int main() {
    // Настройки эксперимента
    vector<int> sizes = { 10, 50, 100, 200, 300, 500, 1000 };
    const int ITERATIONS = 10;
    const double DENSITY = 0.3;

    cout << "Starting Benchmark (Density: " << (DENSITY * 100) << "%)..." << endl;
    cout << fixed << setprecision(3);
    cout << "-----------------------------------------------" << endl;
    cout << "|    N   |  A* Time (ms)  | Floyd-W Time (ms) |" << endl;
    cout << "-----------------------------------------------" << endl;

    for (int n : sizes) {
        double total_time_astar = 0.0;
        double total_time_floyd = 0.0;

        for (int iter = 0; iter < ITERATIONS; ++iter) {
            GraphData graph = GenerateGraph(n, DENSITY);

            random_device rd;
            mt19937 gen(rd());
            uniform_int_distribution<> node_dist(0, n - 1);
            int start = node_dist(gen);
            int goal = node_dist(gen);
            while (start == goal) goal = node_dist(gen);

            // 1. Замер A*
            auto t1 = chrono::high_resolution_clock::now();
            Run_A_Star(start, goal, n, graph.adjList, graph.coords);
            auto t2 = chrono::high_resolution_clock::now();
            total_time_astar += chrono::duration<double, milli>(t2 - t1).count();

            // 2. Замер Флойда-Уоршелла (теперь без ограничений)
            auto t3 = chrono::high_resolution_clock::now();
            Run_Floyd_Warshall(n, graph.adjMat);
            auto t4 = chrono::high_resolution_clock::now();
            total_time_floyd += chrono::duration<double, milli>(t4 - t3).count();
        }

        double avg_astar = total_time_astar / ITERATIONS;
        double avg_floyd = total_time_floyd / ITERATIONS;

        cout << "| " << setw(6) << n << " | "
            << setw(14) << avg_astar << " | "
            << setw(17) << avg_floyd << " |" << endl;
    }
    cout << "-----------------------------------------------" << endl;
    return 0;
}
