from matplotlib import pyplot as plt
from plotter import load_data

def main():
    lat, lon = load_data("data/data_weg.txt")

    fig, ax = plt.subplots()
    xdiff = max(lon) - min(lon)
    ydiff = max(lat) - min(lat)

    im = plt.imread("map_spansberg.png")
    ax.imshow(im, extent=[13.385437, 13.398942, 51.417562, 51.432451])  # exact values taken from google maps
    ax.scatter(lon, lat, marker="o", s=1)
    
    plt.show()

if __name__ == "__main__":
    main()