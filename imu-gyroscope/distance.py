import math

def parse_coordinates(coord_str):
    parts = coord_str.split("\t")
    
    # Extract the numbers and store in a list
    coordinates = [float(p.split(": ")[1]) for p in parts]
    
    return coordinates  # Returns a list [X, Y, Z]

# Example usage
input_str = "X: 267.54\tY: 8.456\tZ: 164.584838399594"
result = parse_coordinates(input_str)

print("Parsed input: ", result, "\n")

def calculate_distance(coord1, coord2):
    # Euclidean distance formula for 3D space
    return math.sqrt((coord2[0] - coord1[0]) ** 2 + (coord2[1] - coord1[1]) ** 2 + (coord2[2] - coord1[2]) ** 2)

# Example usage
coord1 = [267.54, 8.456, 164.584838399594]
coord2 = [300.275, 20.8424, 180.53247864]

distance = calculate_distance(coord1, coord2)
print("Coordinate 1: ", coord1)
print("Coordinate 2: ", coord2)
print("")
print(f"The distance between the two coordinates is: {distance}")