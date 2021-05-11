from plotter import load_data
from statistics import mean, median, variance

def main():
    paths = [f"data/{i}.txt" for i in range(1, 26)]

    with open("data/mean_positions.txt", "w") as f:
        f.write("nr: mean_lat, mean_lon; median_lat; median_lon; var_lat, var_lon\n")
        for i, path in enumerate(paths):
            lat, lon = load_data(path)
            f.write(f"{i + 1}: {round(mean(lat), 5)},\t {round(mean(lon), 5)};\t\t {round(median(lat), 5)},\t {round(median(lon), 5)}\n")

if __name__ == "__main__":
    main()