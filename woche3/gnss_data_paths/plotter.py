import matplotlib.pyplot as plt
import matplotlib

def load_data(path):
    lat = []
    lon = []

    with open(path, "r") as f:
        for i, line in enumerate(f):
            # fisrt line meta data
            if i == 0: continue
            if line.startswith(","): continue

            la, lo = line.strip().split(",")

            la_angle = la[0:2]
            la_angle_minutes = la[2::]

            lo_angle = lo[0:3]
            lo_angle_minutes = lo[3::]
            
            lat.append(float(la_angle) + float(la_angle_minutes) / 60.0)
            lon.append(float(lo_angle) + float(lo_angle_minutes) / 60.0)

    return (lat, lon)

def load_mean_data():
    path = "data/mean_positions.txt"

    mean   = []
    median = []
    with open(path, "r") as f:
        for i, line in enumerate(f):
            if i == 0: continue

            mean_data, median_data = line.split(":")[1].split(";")
            mean_   = map(lambda x: float(x.strip()), mean_data.split(","))
            median_ = map(lambda x: float(x.strip()), median_data.split(","))

            mean.append(mean_)
            median.append(median_)

    return (mean, median)

def plot_raw():
    paths = [f"data/{i}.txt" for i in range(1, 26)]

    for path in paths:
        lat, lon = load_data(path)
        plt.scatter(lon, lat, marker="o")
  
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    
    plt.show()

def plot_mean():
    mean, median = load_mean_data()

    for point in mean:
        lat, lon = point
        plt.scatter(lon, lat, marker="o")
   
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    
    plt.show()

def plot_median():
    mean, median = load_mean_data()

    for point in median:
        lat, lon = point
        plt.scatter(lon, lat, marker="o")
   
    plt.xlabel("longitude")
    plt.ylabel("latitude")
    
    plt.show()


def main():
    mode = input("[r]aw, [m1]ean, [m2]edian: ")

    if   mode == "r":
        plot_raw()
    elif mode == "m1":
        plot_mean()
    elif mode == "m2":
        plot_median()

if __name__ == "__main__":
    main()