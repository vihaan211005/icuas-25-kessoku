import numpy as np
import matplotlib.pyplot as plt

def linear(angle, visited, theta):
    sorted_existing = sorted(visited)
    def overlap(arc, existing):
        start_arc, end_arc = arc
        total = 0
        for s, e in existing:
            assert(e >= s)
            if(s > end_arc):
                break
            if(e < start_arc):
                continue
            start = max(s, start_arc)
            end = min(e, end_arc)
            assert end >= start
            total += end - start
            start_arc = end
            if(start_arc == end_arc):
                break
        return total
    start_range = (angle - theta) % (2 * np.pi)
    end_range = (angle + theta) % (2 * np.pi)
    ans = 0
    if(end_range < start_range):
        ans += overlap((start_range, 2 * np.pi), sorted_existing)
        ans += overlap((0, end_range), sorted_existing)
    else:
        ans += overlap((start_range, end_range), sorted_existing)
    return 1 - (ans / (2 * theta))

def gauss(r, sigma):
    coefficient = 1
    exponent = - (r ** 2) / (2 * sigma ** 2)
    return coefficient * np.exp(exponent)

def funx(x, y, building, sigma, visited, theta):
    bx, by = building
    dx, dy = x - bx, y - by
    r = np.hypot(dx, dy)
    angle =  np.arctan2(dy, dx) % (2 * np.pi)
    p1 = linear(angle, visited, theta)
    p2 = gauss(r, sigma)

    assert p1 * p2 > -1e-6
    return max(p1 * p2, 0)

def make_matrix(buildings, edge, m, n, sigma, theta, visible_arcs_building):
    prob_matrix = np.zeros((m, n))
    for idx, building in enumerate(buildings):
        for i in range(m):
            for j in range(n):
                prob_matrix[i][j] += funx(i * edge, j * edge, building, sigma, visible_arcs_building[idx], theta)
    return prob_matrix

# Plotting and testing
def test_case(building, visited, sigma, theta, title):
    x = np.linspace(-10, 10, 300)
    y = np.linspace(-10, 10, 300)
    X, Y = np.meshgrid(x, y)
    Z = np.vectorize(lambda x, y: funx(x, y, building, sigma, visited, theta))(X, Y)

    plt.figure(figsize=(6, 5))
    plt.contourf(X, Y, Z, levels=100, cmap='viridis')
    plt.colorbar(label='funx(x, y)')
    plt.scatter(*building, color='red', label='Building')
    plt.title(title)
    plt.xlabel("x")
    plt.ylabel("y")
    plt.legend()
    plt.axis('equal')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # Test cases
    test_case(building=(0, 0),
            visited=[(0, np.pi/4), (np.pi, 5*np.pi/4)],
            sigma=2,
            theta=np.pi/6,
            title="Case 1: Some partial visibility")

    test_case(building=(0, 0),
            visited=[(0, 2*np.pi)],
            sigma=2,
            theta=np.pi/6,
            title="Case 2: Fully covered angles (should be max value near building)")

    test_case(building=(0, 0),
            visited=[],
            sigma=2,
            theta=np.pi/6,
            title="Case 3: No visibility (output should be zero)")

    test_case(building=(2, 2),
            visited=[(0, np.pi)],
            sigma=3,
            theta=np.pi/4,
            title="Case 4: Half-circle visibility, shifted building")