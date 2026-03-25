explain the most important parts of this code in a few bullet points 
# -------------------------------
# HM01B0 Blob Detection (Optimized)
# -------------------------------

WIDTH = 96    # downsampled for speed (you can use 160 or 320 later)
HEIGHT = 96

THRESHOLD = 120     # tune this!
MIN_BLOB_SIZE = 50  # ignore noise

LEFT_BOUND = WIDTH // 3
RIGHT_BOUND = 2 * WIDTH // 3

def process_frame(frame):
    """
    frame: 2D array [HEIGHT][WIDTH] grayscale (0–255)
    """

    visited = [[0]*WIDTH for _ in range(HEIGHT)]
    blobs = []

    for y in range(HEIGHT):
        for x in range(WIDTH):

            if frame[y][x] > THRESHOLD and not visited[y][x]:

                # Flood fill (BFS)
                stack = [(x, y)]
                visited[y][x] = 1

                pixels = []
                area = 0

                while stack:
                    cx, cy = stack.pop()
                    pixels.append((cx, cy))
                    area += 1

                    # 4-connected neighbors
                    for nx, ny in [(cx+1,cy),(cx-1,cy),(cx,cy+1),(cx,cy-1)]:
                        if 0 <= nx < WIDTH and 0 <= ny < HEIGHT:
                            if frame[ny][nx] > THRESHOLD and not visited[ny][nx]:
                                visited[ny][nx] = 1
                                stack.append((nx, ny))

                if area > MIN_BLOB_SIZE:
                    blobs.append(pixels)

    if not blobs:
        return None, None

    # Pick largest blob
    largest_blob = max(blobs, key=len)

    # Compute centroid
    sum_x = sum(p[0] for p in largest_blob)
    sum_y = sum(p[1] for p in largest_blob)
    N = len(largest_blob)

    cx = sum_x // N
    cy = sum_y // N

    return cx, cy


def get_direction(cx):
    if cx is None:
        return "SEARCH"

    if cx < LEFT_BOUND:
        return "LEFT"
    elif cx > RIGHT_BOUND:
        return "RIGHT"
    else:
        return "FORWARD"

