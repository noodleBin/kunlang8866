      
import argparse
from pyproj import Proj

def utm_to_latlon(utm_x, utm_y, utm_zone=51, is_northern=True):
    proj_utm = Proj(proj="utm", zone=utm_zone, ellps="WGS84", south=not is_northern)
    lon, lat = proj_utm(utm_x, utm_y, inverse=True)
    return lat, lon

def main():
    parser = argparse.ArgumentParser(description="Convert UTM coordinates to latitude and longitude.")
    parser.add_argument("--utm_x", type=float, required=True, help="UTM X coordinate")
    parser.add_argument("--utm_y", type=float, required=True, help="UTM Y coordinate")

    args = parser.parse_args()
    args.utm_x += 250932.85
    args.utm_y += 3987498.59
    lat, lon = utm_to_latlon(args.utm_x, args.utm_y)
    print(f"lon: {lon:.8f}, lat: {lat:.8f}")

if __name__ == "__main__":
    main()

    