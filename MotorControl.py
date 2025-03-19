import heapq
import time


# Node sinfi – hər düyünün koordinatlarını, G, H, F dəyərlərini və parent göstəricisini saxlayır.
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.g = float('inf')  # Başlanğıc nöqtədən bu nöqtəyə qədər real məsafə (ilk olaraq sonsuz)
        self.h = 0  # Bu nöqtədən hədəfə qədər təxmini məsafə (evristika)
        self.f = 0  # f = g + h
        self.parent = None  # Yolun bərpası üçün valideyn

    def __lt__(self, other):
        return self.f < other.f


# Evristika funksiyası – Manhattan məsafəsi hesablanır.
def heuristic(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


# Qonşu düyünləri tapmaq – 4 istiqamət (yuxarı, aşağı, sol, sağ)
def get_neighbors(node, grid):
    neighbors = []
    rows = len(grid)
    cols = len(grid[0])
    directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
    for dx, dy in directions:
        nx = node.x + dx
        ny = node.y + dy
        if 0 <= nx < rows and 0 <= ny < cols:
            if grid[nx][ny] == 1:  # 1 = keçilən sahə
                neighbors.append((nx, ny))
    return neighbors


# A* alqoritmi – labirintdə ən qısa yolu tapır.
def a_star(grid, start, goal):
    rows = len(grid)
    cols = len(grid[0])

    # Hər hücrə üçün Node yaradılır.
    node_grid = [[Node(x, y) for y in range(cols)] for x in range(rows)]

    start_node = node_grid[start[0]][start[1]]
    goal_node = node_grid[goal[0]][goal[1]]

    # Başlanğıc düyünü üçün ilkin dəyərləri təyin edirik.
    start_node.g = 0
    start_node.h = heuristic(start_node, goal_node)
    start_node.f = start_node.h

    open_list = []
    heapq.heappush(open_list, start_node)
    closed_set = set()

    print("A* axtarışı başladı...\n")

    # A* axtarışı – open_list boş olana qədər davam edir.
    while open_list:
        current = heapq.heappop(open_list)
        print(f"İndi işlənən düyün: ({current.x}, {current.y}), F: {current.f}, G: {current.g}, H: {current.h}")

        # Əgər hədəfə çatmışıqsa, yolu bərpa edirik.
        if current.x == goal_node.x and current.y == goal_node.y:
            print("\nHədəf tapıldı!")
            return reconstruct_path(current)

        closed_set.add((current.x, current.y))

        # Qonşu düyünləri araşdırırıq.
        for (nx, ny) in get_neighbors(current, grid):
            if (nx, ny) in closed_set:
                continue

            neighbor = node_grid[nx][ny]
            tentative_g = current.g + 1  # Hər addımın xərci 1 qəbul edilir.

            # Əgər yeni yol daha yaxşıdırsa, yeniləyirik.
            if tentative_g < neighbor.g:
                neighbor.parent = current
                neighbor.g = tentative_g
                neighbor.h = heuristic(neighbor, goal_node)
                neighbor.f = neighbor.g + neighbor.h
                heapq.heappush(open_list, neighbor)
                print(f"  --> Qonşu əlavə edildi: ({neighbor.x}, {neighbor.y}), F: {neighbor.f}")

        print()  # Addım arası boş sətir, axtarış prosesini ayırır.
        # Demo məqsədilə qısa gecikmə (şərhə alınmışdır)
        # time.sleep(0.5)

    return None


# Tapılmış yolu bərpa edən funksiya (geri izləmə)
def reconstruct_path(end_node):
    path = []
    current = end_node
    while current:
        path.append((current.x, current.y))
        current = current.parent
    path.reverse()
    return path


# Robot hərəkətinin simulyasiyası: tapılan yol boyunca addım-addım irəliləyir.
def simulate_robot_movement(path):
    print("\nRobot hərəkəti başlayır:")
    for pos in path:
        print(f"Robot mövqeyi: {pos}")
        # Həqiqi tətbiqdə burada robotu hərəkətə salmaq üçün sensorlar və motor əmrləri göndərilə bilər.
        # time.sleep(0.3)  # Simulyasiya üçün gecikmə əlavə edilə bilər.
    print("Robot hədəfə çatdı!")


# Əsas proqram: labirintin yaradılması, A* axtarışı və robotun hərəkəti.
if __name__ == "__main__":
    # Labirint: 1 = keçilən, 0 = maneə
    maze = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 0, 1],
        [1, 0, 1, 1, 1],
        [1, 1, 1, 0, 1]
    ]

    # Case 1: Robot labirintin girişində yerləşdirilir.
    start = (0, 0)  # Labirintin sol-yuxarı küncü giriş kimi qəbul edilir.
    goal = (4, 4)  # Hədəf nöqtəsi labirintin çıxışı kimi qəbul edilir.

    print("Labirint:")
    for row in maze:
        print(row)

    # Case 2: A* alqoritmi işə salınır və ən qısa yol tapılır.
    path = a_star(maze, start, goal)

    if path:
        print("\nTapılan yol:")
        print(path)
        # Case 3: Robot tapılan yol üzrə hərəkət edir.
        simulate_robot_movement(path)
    else:
        print("Hədəfə yol tapılmadı!")
