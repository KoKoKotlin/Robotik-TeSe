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

def main():
    path = input("Path to data: ")

    lat, lon = load_data(path)
    plt.xlabel("latitude")
    plt.ylabel("longitude")
    
    plt.scatter(lat, lon, marker="o")
    # plt.axis([round(min(lat), 5), round(max(lat), 5), round(min(lon), 5), round(max(lon), 5)]) 
    
    plt.show()

if __name__ == "__main__":
    main()