from matplotlib import pyplot as plt
from plotter import load_data

def main():
    lat, lon = load_data("data/data_weg.txt")

    fig, ax = plt.subplots()
    xdiff = max(lon) - min(lon)
    ydiff = max(lat) - min(lat)

    im = plt.imread("map_spansberg.png")
    ax.imshow(im, extent=[min(lon) - xdiff * 0.01, max(lon), min(lat) - ydiff * 0.04, max(lat) + ydiff * 0.035])
    ax.scatter(lon, lat, marker="o", s=2)
    
    figManager = plt.get_current_fig_manager()
    figManager.resize(*figManager.window.maxsize())
    
    plt.show()

if __name__ == "__main__":
    main()