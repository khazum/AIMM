import sys
import xml.etree.ElementTree as ET
import json
import math # Needed for NaN if we choose to use it, though None->null is fine.

def parse_gpx(gpx_path):
    """
    Parses a GPX file and returns a list of (latitude, longitude, altitude).
    Handles potential parsing errors.
    """
    try:
        tree = ET.parse(gpx_path)
        root = tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing GPX file {gpx_path}: {e}", file=sys.stderr)
        return None # Indicate failure
    except FileNotFoundError:
        print(f"Error: GPX file not found at {gpx_path}", file=sys.stderr)
        return None # Indicate failure

    # Determine if there's a namespace
    ns = None
    if root.tag.startswith("{") and "}gpx" in root.tag:
        ns = root.tag.split("}")[0].strip("{")

    # Helper function to handle namespaced tags
    def nstag(tag):
        if ns:
            return f"{{{ns}}}{tag}"
        return tag

    coords = []
    # Look for <trkpt lat="..." lon="..."> elements
    # Using root.findall('.//' + nstag('trkpt')) is slightly more explicit
    # for finding descendants than root.iter, but iter works fine too.
    for trkpt in root.iter(nstag("trkpt")):
        try:
            lat = float(trkpt.attrib["lat"])
            lon = float(trkpt.attrib["lon"])
            ele_elem = trkpt.find(nstag("ele"))
            alt = 0.0 # Default altitude
            if ele_elem is not None and ele_elem.text is not None:
                 try:
                     alt = float(ele_elem.text)
                 except ValueError:
                     # Ignore invalid altitude text, keep default 0.0
                     print(f"Warning: Invalid altitude '{ele_elem.text}' found, using 0.0.", file=sys.stderr)
                     pass # Keep alt = 0.0
            coords.append((lat, lon, alt))
        except KeyError as e:
            print(f"Warning: Skipping trackpoint missing required attribute: {e}", file=sys.stderr)
            continue # Skip this point
        except ValueError as e:
            print(f"Warning: Skipping trackpoint with invalid coordinate value: {e}", file=sys.stderr)
            continue # Skip this point

    return coords

def filter_coords_by_bbox(coords, lat1, lon1, lat2, lon2):
    """
    Filters out any (lat, lon, alt) that lies outside the bounding box
    defined by two opposite corners: (lat1, lon1) and (lat2, lon2).

    Returns a new list of coords within the bounding box.
    """
    # Ensure coords is not None before proceeding
    if coords is None:
        return []

    min_lat = min(lat1, lat2)
    max_lat = max(lat1, lat2)
    min_lon = min(lon1, lon2)
    max_lon = max(lon1, lon2)

    filtered = []
    for (lat, lon, alt) in coords:
        if min_lat <= lat <= max_lat and min_lon <= lon <= max_lon:
            filtered.append((lat, lon, alt))
    return filtered

def create_plan_json(coords, step=1, default_alt=0.0, acceptance_radius=2.0):
    """
    Builds a QGroundControl .plan JSON structure using every 'step'-th coordinate.
    Altitude is set to 'default_alt' for a rover scenario.
    Acceptance radius defines how close the vehicle must get to the waypoint.

    Returns a Python dict ready to be saved as JSON, or None if coords is empty.
    """
    if not coords:
        return None # Cannot create a plan with no coordinates

    items = []
    do_jump_id = 1

    # Determine planned home position from the first point in the filtered list
    first_lat, first_lon, _ = coords[0]
    # Use default_alt for home altitude matching the mission altitude frame/mode
    planned_home = [first_lat, first_lon, default_alt]

    for i in range(0, len(coords), step):
        lat, lon, gpx_alt = coords[i]
        # For a rover, we generally ignore actual altitude and just use a default.
        alt = default_alt

        item = {
            "AMSLAltAboveTerrain": None,
            "Altitude": alt,
            "AltitudeMode": 1,       # 0=Absolute, 1=Relative to Home
            "autoContinue": True,
            "command": 16,          # MAV_CMD_NAV_WAYPOINT
            "doJumpId": do_jump_id,
            "frame": 3,             # MAV_FRAME_GLOBAL_RELATIVE_ALT
            "params": [
                0.0,                   # param1: Hold time in seconds
                acceptance_radius,     # param2: Acceptance radius in meters (common default: 0 = default, here we set explicitly)
                0.0,                   # param3: Pass-through radius (0 to pass through WP, >0 to orbit)
                None,                  # param4: Desired Yaw angle at waypoint (None/NaN for unchanged) -> JSON null
                lat,                   # param5: Latitude
                lon,                   # param6: Longitude
                alt                    # param7: Altitude
            ],
            "type": "SimpleItem"
        }
        items.append(item)
        do_jump_id += 1

    # Ensure there's at least one item if coords was not empty
    if not items:
         print("Warning: No waypoints generated after applying step filter.", file=sys.stderr)
         return None


    plan = {
        "fileType": "Plan",
        "groundStation": "QGroundControl",
        "version": 1,
        "mission": {
            "cruiseSpeed": 5,       # Default, could be parameter
            "firmwareType": 4,      # 4=ArduRover
            "hoverSpeed": 5,        # Default, could be parameter (less relevant for rover)
            "items": items,
            "plannedHomePosition": planned_home, # Use the derived home position
            "vehicleType": 4,       # 4=Rover
            "version": 2
        },
        "geoFence": {
            "circles": [],
            "polygons": [],
            "version": 1
        },
        "rallyPoints": {
            "points": [],
            "version": 1
        }
    }

    return plan

def main():
    if len(sys.argv) < 8:
        print("Usage: python create_plan_from_gpx_bounding_corners.py <input.gpx> <step> <lat1> <lon1> <lat2> <lon2> <output.plan>")
        print("Example: python create_plan_from_gpx_bounding_corners.py track.gpx 10 41.70 -85.03 41.71 -85.02 rover_mission.plan")
        sys.exit(1)

    gpx_file = sys.argv[1]
    output_plan = sys.argv[7]

    # Validate numeric inputs
    try:
        step = int(sys.argv[2])
        if step < 1:
            print("Error: Step must be a positive integer.", file=sys.stderr)
            sys.exit(1)
        lat1 = float(sys.argv[3])
        lon1 = float(sys.argv[4])
        lat2 = float(sys.argv[5])
        lon2 = float(sys.argv[6])
    except ValueError as e:
        print(f"Error: Invalid numeric argument: {e}", file=sys.stderr)
        sys.exit(1)

    # 1) Parse GPX (includes basic file/parse error handling now)
    coords = parse_gpx(gpx_file)
    if coords is None: # Check if parsing failed
        sys.exit(2)
    if not coords:
        print(f"No valid GPX trackpoints found in {gpx_file}")
        sys.exit(2)
    print(f"Successfully parsed {len(coords)} points from {gpx_file}.")

    # 2) Filter out points outside the bounding box
    coords_in_box = filter_coords_by_bbox(coords, lat1, lon1, lat2, lon2)
    if not coords_in_box:
        print("No points found within the specified bounding box.")
        # Decide if this is an error or just an outcome
        # Let's exit gracefully without creating an empty file
        sys.exit(0) # Exit code 0 indicating normal termination, just no points.
    print(f"Filtered down to {len(coords_in_box)} points within the bounding box.")


    # 3) Build the QGC .plan JSON with the filtered points (stepped).
    # Can add acceptance_radius as parameter later if needed
    plan_data = create_plan_json(coords_in_box, step=step, default_alt=0.0, acceptance_radius=2.0)

    if plan_data is None:
         print("Failed to generate plan data (likely no points after stepping).")
         sys.exit(4)

    # 4) Save to .plan
    try:
        with open(output_plan, 'w') as f:
            json.dump(plan_data, f, indent=4)
    except IOError as e:
        print(f"Error writing output file {output_plan}: {e}", file=sys.stderr)
        sys.exit(5)

    num_waypoints = len(plan_data['mission']['items'])
    print(f"\nSuccessfully created QGC .plan file: {output_plan}")
    print(f"Included {num_waypoints} waypoints (every {step}-th point within the box).")
    print(f"Bounding Box Corners: ({lat1}, {lon1}), ({lat2}, {lon2})")
    print(f"Planned Home Position set near first waypoint: {plan_data['mission']['plannedHomePosition']}")


if __name__ == "__main__":
    main()
