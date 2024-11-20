#include <iostream>
#include <vector>
#include <climits>
#include <fstream>
#include <queue>
#include <random>
#include <chrono>

using namespace std;

#define para std::pair<long long, long long>

int V; // Number of vertices
double G; // Graph density
vector<vector<int>> tab; // Adjacency matrix
vector<vector<pair<int, int>>> adjList; // Adjacency list

void shuffle(const string& filename) {
    ofstream file(filename);
    if (!file) {
        cerr << "Blad podczas otwierania pliku do zapisu: " << filename << endl;
        return;
    }
    mt19937 rng{random_device{}()};
    uniform_int_distribution<> dist(0, V - 1);

    vector<pair<int, int>> edges;
    int edgeCount = static_cast<int>(G * V * (V - 1) / 200);
    file << V << " " << edgeCount << endl;
    for (int i = 0; i < edgeCount; i++) {
        int src = dist(rng);
        int dest = dist(rng);
        while (dest == src) {
            dest = dist(rng);
        }
        int weight = dist(rng) % 100 + 1; // Ensure weight is non-zero
        edges.emplace_back(src, dest);
        file << src << " " << dest << " " << weight << endl;
    }
    file.close();
}

void load_L(const string& filename) {
    ifstream file(filename);
    if (!file) {
        cerr << "Blad podczas otwierania pliku do odczytu: " << filename << endl;
        return;
    }
    int edges;
    file >> V >> edges;
    adjList.clear();
    adjList.resize(V);
    for (int i = 0; i < edges; i++) {
        int src, dest, weight;
        file >> src >> dest >> weight;
        adjList[src].emplace_back(dest, weight);
    }
    file.close();
}

void load_M(const string& filename) {
    ifstream file(filename);
    if (!file) {
        cerr << "Blad podczas otwierania pliku do odczytu: " << filename << endl;
        return;
    }
    int edges;
    file >> V >> edges;
    tab.assign(V, vector<int>(V, 0));
    for (int i = 0; i < edges; i++) {
        int src, dest, weight;
        file >> src >> dest >> weight;
        tab[src][dest] = weight;
    }
    file.close();
}

void printGraph_L() {
    cout << "Reprezentacja listy sasiedztwa:" << endl;
    for (int i = 0; i < V; i++) {
        cout << i << ": ";
        for (const auto& edge : adjList[i]) {
            cout << "(" << edge.first << ", " << edge.second << ") ";
        }
        cout << endl;
    }
}

void printGraph_M() {
    cout << "Reprezentacja macierzy sasiedztwa:" << endl;
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            cout << tab[i][j] << " ";
        }
        cout << endl;
    }
}

void Dijkstra_L(int s) {
    vector<int> d(V, INT_MAX);
    d[s] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, s});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (const auto& edge : adjList[u]) {
            int v = edge.first;
            int weight = edge.second;

            if (d[u] + weight < d[v]) {
                d[v] = d[u] + weight;
                pq.push({d[v], v});
            }
        }
    }

    cout << "Najkrotsze odleglosci od wierzcholka " << s << ":\n";
    for (int i = 0; i < V; i++) {
        if (d[i] == INT_MAX)
            cout << "wierzcholek " << i << " is unreachable\n";
        else
            cout << "wierzcholek " << i << ": " << d[i] << endl;
    }
}

void Dijkstra_M(int s) {
    vector<int> d(V, INT_MAX);
    d[s] = 0;
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;
    pq.push({0, s});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (int v = 0; v < V; v++) {
            if (tab[u][v] && d[u] + tab[u][v] < d[v]) {
                d[v] = d[u] + tab[u][v];
                pq.push({d[v], v});
            }
        }
    }

    cout << "Najkrotsze odleglosci od wierzcholka " << s << ":\n";
    for (int i = 0; i < V; i++) {
        if (d[i] == INT_MAX)
            cout << "wierzcholek " << i << " is unreachable\n";
        else
            cout << "wierzcholek " << i << ": " << d[i] << endl;
    }
}
int main() {
    int wybor;
    string nazwa;

    while (true) {
        cout << "Menu wyboru:" << endl;
        cout << "  1 - Wygeneruj graf" << endl;
        cout << "  2 - Wczytaj graf i wyswietl(listowo/macierzowo)" << endl;
        cout << "  3 - Usredniony czas wykonywania programu dla 100 powtorzen" << endl;
        cout << "  4 - Uruchom algorytm Dijkstry i wyswietl wyniki" << endl;
        cout << "------------------------------------" << endl;
        cin >> wybor;

        switch (wybor) {
            case 1:
                cout << "Podaj ilosc wierzcholkow grafu: " << endl;
                cin >> V;
                if (V <= 0) {
                    cerr << "Liczba wierzcholkow powinna byc dodatnia." << endl;
                    break;
                }
                cout << "Podaj gestosc grafu (0-100%): " << endl;
                cin >> G;
                if (G < 0 || G > 100) {
                    cerr << "Gestosc grafu powinna byc pomiedzy 0 a 100." << endl;
                    break;
                }
                cout << "Wpisz nazwe pliku: " << endl;
                cin >> nazwa;
                shuffle(nazwa);
                cout << "Graf zostal wygenerowany" << endl;
                break;
            case 2:
                cout << "Podaj nazwe pliku: " << endl;
                cin >> nazwa;
                load_L(nazwa);
                load_M(nazwa);
                cout << "Pomyslnie wczytano dane z pliku\n" << endl;
                printGraph_L();
                printGraph_M();
                break;
            case 3: {
                int s;
                cout << "Podaj nazwe pliku: " << endl;
                cin >> nazwa;
                cout << "Podaj wierzcholek poczatkowy: " << endl;
                cin >> s;
                if (s < 0 || s >= V) {
                    cerr << "Nieprawidlowy wierzcholek poczatkowy." << endl;
                    break;
                }
                double czasL = 0, czasM = 0;
                for (int i = 0; i < 100; i++) {
                    shuffle(nazwa);
                    load_L(nazwa);
                    load_M(nazwa);
                    auto start = chrono::high_resolution_clock::now();
                    Dijkstra_L(s);
                    auto stop = chrono::high_resolution_clock::now();
                    czasL += chrono::duration_cast<chrono::microseconds>(stop - start).count();
                    start = chrono::high_resolution_clock::now();
                    Dijkstra_M(s);
                    stop = chrono::high_resolution_clock::now();
                    czasM += chrono::duration_cast<chrono::microseconds>(stop - start).count();
                }
                cout << "Sredni czas dla reprezentacji listowej: " << czasL / 100000 << " s" << endl;
                cout << "Sredni czas dla reprezentacji macierzowej: " << czasM / 100000 << " s" << endl;
                break;
            }
            case 4: {
                int s;
                cout << "Podaj nazwe pliku: " << endl;
                cin >> nazwa;
                load_L(nazwa);
                load_M(nazwa);
                cout << "Podaj wierzcholek poczatkowy: " << endl;
                cin >> s;
                if (s < 0 || s >= V) {
                    cerr << "Nieprawidlowy wierzcholek poczatkowy." << endl;
                    break;
                }
                cout << "Wyniki dla reprezentacji listowej:" << endl;
                Dijkstra_L(s);
                cout << "Wyniki dla reprezentacji macierzowej:" << endl;
                Dijkstra_M(s);
                break;
            }
            default:
                cout << "Nieprawidlowy wybor." << endl;
                break;
        }
    }
    return 0;
}