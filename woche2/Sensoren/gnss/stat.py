from statistics import mean, variance
from plotter import load_data

def main():
    path = input("Path to data: ")
    lat, lon = load_data(path)

    lat_mean = mean(lat)
    lon_mean = mean(lon)

    lat_var = variance(lat)
    lon_var = variance(lon)
    
    print(f"lat mean: {round(lat_mean, 4)} lon mean: {round(lon_mean, 4)}\nlat variance: {lat_var} lon variance: {lon_var}")

if __name__ == "__main__":
    main()