import matplotlib.pyplot as plt

MAX = 50


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return f'({self.x}, {self.y})'

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def to_tuple(self):
        return self.x, self.y

    # Up, Right, Down, Left
    def neighbors(self):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        result = []
        for dx, dy in directions:
            nx = self.x + dx
            ny = self.y + dy
            if 0 <= nx < MAX and 0 <= ny < MAX:
                result.append(Point(nx, ny))
        return result


#Drawing Functions

def draw_board():
    fig = plt.figure(figsize=[8, 8])
    ax = fig.add_subplot(111)
    ax.set_axis_off()
    return fig, ax


def draw_grids(ax):
    for x in range(MAX):
        ax.plot([x, x], [0, MAX - 1], color='0.75', linestyle='dotted')
    for y in range(MAX):
        ax.plot([0, MAX - 1], [y, y], color='0.75', linestyle='dotted')
    ax.set_position([0, 0.02, 1, 1])


def draw_point(ax, x, y):
    ax.plot(x, y, 'o', markersize=4,
            markeredgecolor='k',
            markerfacecolor='k')


def draw_source(ax, x, y):
    ax.plot(x, y, 'o', markersize=4,
            markeredgecolor='b',
            markerfacecolor='b')


def draw_dest(ax, x, y):
    ax.plot(x, y, 'o', markersize=4,
            markeredgecolor='r',
            markerfacecolor='r')


def draw_green_point(ax, x, y):
    ax.plot(x, y, 'o', markersize=4,
            markeredgecolor='g',
            markerfacecolor='g')


def draw_line(ax, xs, ys):
    ax.plot(xs, ys, color='k')


def draw_green_line(ax, xs, ys):
    ax.plot(xs, ys, color='g')


def draw_result_line(ax, xs, ys):
    ax.plot(xs, ys, color='r')


#Geometry

def point_on_segment(px, py, x1, y1, x2, y2):
    if min(x1, x2) <= px <= max(x1, x2) and min(y1, y2) <= py <= max(y1, y2):
        dx1 = px - x1
        dy1 = py - y1
        dx2 = x2 - x1
        dy2 = y2 - y1
        return dx1 * dy2 == dy1 * dx2
    return False


def point_in_polygon(point, polygon):
    x, y = point.x, point.y
    inside = False
    n = len(polygon)

    for i in range(n):
        x1, y1 = polygon[i].x, polygon[i].y
        x2, y2 = polygon[(i + 1) % n].x, polygon[(i + 1) % n].y

        # If exactly on boundary, treat as inside
        if point_on_segment(x, y, x1, y1, x2, y2):
            return True

        if ((y1 > y) != (y2 > y)):
            xinters = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < xinters:
                inside = not inside

    return inside



def action_cost(point, enclosures, turfs):
    for poly in enclosures:
        if point_in_polygon(point, poly):
            return None  # blocked

    for poly in turfs:
        if point_in_polygon(point, poly):
            return 1.5

    return 1
