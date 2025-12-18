#include <iostream>
#include <vector>
#include <limits>
#include <iomanip> 

using namespace std; // Добавлена директива для упрощения кода

// Определяем константу "Бесконечность"
const double Infinity = numeric_limits<double>::infinity();

// Структура для представления ребра
struct Edge {
    int u; // Откуда
    int v; // Куда
    double weight; // Вес
};

vector<vector<double>> Floyd_Warshall(int N, const vector<Edge>& edges) {

    // ЭТАП 1: ИНИЦИАЛИЗАЦИЯ МАТРИЦЫ

    // Создать матрицу S размером N на N
    // Изначально заполняем всё Бесконечностью
    vector<vector<double>> S(N, vector<double>(N, Infinity));

    // 1. Устанавливаем 0 на диагонали (i == j)
    for (int i = 0; i < N; ++i) {
        S[i][i] = 0.0;
    }

    // 2. Заполняем веса ребер
    for (const auto& edge : edges) {
        // Предполагаем, что вершины проиндексированы 0..(N-1)
        S[edge.u][edge.v] = edge.weight;
    }

    // ЭТАП 2: ТРОЙНОЙ ЦИКЛ (ДИНАМИЧЕСКОЕ ПРОГРАММИРОВАНИЕ) 

    // k - промежуточная вершина
    for (int k = 0; k < N; ++k) {

        // i - начальная вершина
        for (int i = 0; i < N; ++i) {

            // j - конечная вершина
            for (int j = 0; j < N; ++j) {

                // ЭТАП 3: РЕЛАКСАЦИЯ (ПРОВЕРКА ЛУЧШЕГО ПУТИ)

                // Проверка: если пути i->k и k->j существуют
                if (S[i][k] != Infinity && S[k][j] != Infinity) {

                    // Если путь через k короче прямого пути i->j
                    if (S[i][k] + S[k][j] < S[i][j]) {

                        // Обновляем кратчайшее расстояние
                        S[i][j] = S[i][k] + S[k][j];
                    }
                }
            }
        }
    }

    // ЭТАП 4: ВОЗВРАТ РЕЗУЛЬТАТА 
    return S;
}


int main() {

    setlocale(LC_CTYPE, "rus");
    int N = 4; // Количество вершин (0, 1, 2, 3)

    vector<Edge> edges = {
        {0, 1, 5.0},
        {0, 3, 10.0},
        {1, 2, 3.0},
        {2, 3, 1.0}
    };

    vector<vector<double>> resultMatrix = Floyd_Warshall(N, edges);

    // Вывод результата
    cout << "Матрица кратчайших путей S:" << endl;
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (resultMatrix[i][j] == Infinity) {
                cout << setw(5) << "INF";
            }
            else {
                cout << setw(5) << resultMatrix[i][j];
            }
        }
        cout << endl;
    }

    return 0;
}