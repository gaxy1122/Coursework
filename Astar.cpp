#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <algorithm>
#include <limits>

using namespace std;

// Используем бесконечность для инициализации
const double Infinity = numeric_limits<double>::infinity();

// Структура для представления вершины (координаты нужны для эвристики)
struct Node {
    int id;
    double x, y;

    // Операторы сравнения нужны для использования Node как ключа в std::map
    bool operator<(const Node& other) const { return id < other.id; }
    bool operator==(const Node& other) const { return id == other.id; }
};

// Тип для графа: id вершины -> список пар {сосед, вес}
using Graph = std::map<int, std::vector<std::pair<Node, double>>>;

// Функция Эвристики (например, Евклидово расстояние)
double Heuristic(const Node& start, const Node& end) {
    return sqrt(pow(start.x - end.x, 2) + pow(start.y - end.y, 2));
}

std::vector<Node> A_Star(Node start, Node goal, Graph& graph) {

    // ЭТАП 1: ПОДГОТОВКА ДАННЫХ

    // Очередь вершин, которые нужно проверить (Priority Queue)
    // Хранит пары {F-стоимость, Вершина}. Сортировка по возрастанию F.
    // std::greater обеспечивает, что наверху будет элемент с минимальным F
    using Element = pair<double, Node>;
    priority_queue<Element, vector<Element>, greater<Element>> open_set;

    // Содержит только Начало
    open_set.push({ 0.0, start });

    // Словарь "Откуда пришли", чтобы запомнить маршрут
    map<Node, Node> came_from;

    // G: Стоимость пути от начала до конкретной точки
    // Заполняем бесконечностью (через проверку наличия ключа в map)
    map<Node, double> g_score;
    g_score[start] = 0.0;

    // F: Полная оценка (G + Эвристика)
    map<Node, double> f_score;
    f_score[start] = Heuristic(start, goal);

    // Вспомогательный набор для быстрой проверки "ЕСЛИ Сосед не в Открытый_Список"
    // (std::priority_queue не поддерживает быстрый поиск)
    map<int, bool> in_open_set;
    in_open_set[start.id] = true;

    // ЭТАП 2: ОСНОВНОЙ ЦИКЛ ПОИСКА 

    // ПОКА Открытый_Список не пуст:
    while (!open_set.empty()) {

        // 1. Выбираем вершину с самым низким F в списке
        Node current = open_set.top().second;
        open_set.pop(); // Удаляем текущую вершину из списка
        in_open_set[current.id] = false; // Помечаем, что её больше нет в очереди

        // 2. ПРОВЕРКА НА УСПЕХ (Если дошли до финиша)
        // ЕСЛИ Текущая == Конец:
        if (current == goal) {

            // ЭТАП 3: ВОССТАНОВЛЕНИЕ ПУТИ
            std::vector<Node> path;

            // Пока Текущая присутствует в словаре Родители:
            while (came_from.find(current) != came_from.end()) {
                path.push_back(current);        // Добавить Текущая в начало списка
                current = came_from[current];   // Текущая = Родители[Текущая] (Шагаем назад)
            }

            path.push_back(start); // Добавить Начало в начало списка Путь
            std::reverse(path.begin(), path.end()); // Разворачиваем, так как собирали с конца

            return path; // ВЕРНУТЬ Путь
        }

        // 3. ОБРАБОТКА СОСЕДЕЙ
        // ДЛЯ КАЖДОГО Соседа текущей вершины:
        // (graph[current.id] возвращает список соседей)
        if (graph.find(current.id) != graph.end()) {
            for (auto& edge : graph[current.id]) {
                Node neighbor = edge.first;
                double dist_to_neighbor = edge.second; // Расстояние(Текущая, Сосед)

                // Рассчитываем стоимость пути до соседа через текущую точку
                // Новый_G = G_Стоимость[Текущая] + Расстояние
                double tentative_g = g_score[current] + dist_to_neighbor;

                // Получаем текущий G соседа (если его нет — считаем Бесконечностью)
                double neighbor_g = (g_score.count(neighbor)) ? g_score[neighbor] : Infinity;

                // ЕСЛИ Новый_G < G_Стоимость[Сосед]:
                if (tentative_g < neighbor_g) {

                    // Обновляем информацию о соседе
                    came_from[neighbor] = current;         // Родители[Сосед] = Текущая
                    g_score[neighbor] = tentative_g;       // G_Стоимость[Сосед] = Новый_G

                    // Считаем F = G + H
                    double f_val = tentative_g + Heuristic(neighbor, goal);
                    f_score[neighbor] = f_val;

                    // ЕСЛИ Сосед не в Открытый_Список:
                    if (!in_open_set[neighbor.id]) {
                        // Добавить Сосед в Открытый_Список
                        open_set.push({ f_val, neighbor });
                        in_open_set[neighbor.id] = true;
                    }
                }
            }
        }
    }

    // ЭТАП 4: ЗАВЕРШЕНИЕ (ЕСЛИ ПУТЬ НЕ НАЙДЕН) 
    // ВЕРНУТЬ Ошибка (Пустой путь)
    return {};
}

int main() {
    setlocale(LC_CTYPE, "rus");
    // Создаем вершины
    Node n1{ 1, 0, 0 };
    Node n2{ 2, 1, 0 }; // Сосед 1
    Node n3{ 3, 0, 1 }; // Сосед 2 (Стена?)
    Node n4{ 4, 2, 2 }; // Цель

    // Создаем Граф
    Graph graph;
    // Ребра от n1
    graph[1].push_back({ n2, 1.0 }); // Вес 1
    graph[1].push_back({ n3, 10.0 }); // Вес 10 (тяжелый путь)

    // Ребра от n2
    graph[2].push_back({ n4, 1.0 }); // Путь через n2 короткий (1+1=2)

    // Ребра от n3
    graph[3].push_back({ n4, 1.0 }); // Путь через n3 длинный (10+1=11)

    // Запуск A*
    vector<Node> path = A_Star(n1, n4, graph);

    if (!path.empty()) {
        cout << "Путь найден: ";
        for (const auto& node : path) {
            cout << node.id << " ";
        }
        cout << endl;
    }
    else {
        cout << "Путь не существует" << endl;
    }

    return 0;
}