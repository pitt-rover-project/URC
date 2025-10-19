import math
from typing import Optional, Tuple, List
from dataclasses import dataclass
from enum import Enum

class NavigationStatus(Enum):
    """Status codes for navigation state"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    NO_GPS = "no_gps"
    ERROR = "error"

@dataclass
class Coordinate:
    """Represents a GPS coordinate in decimal degrees"""
    latitude: float
    longitude: float
    
    def __repr__(self):
        return f"({self.latitude:.6f}, {self.longitude:.6f})"

class GPSDataParser:
    """
    Parses NMEA sentences to extract SNR values, compute their average, and
    extract coordinates from supported sentence types (GAGSV/GBGSV for SNR
    and GNGLL for coordinates).
    """
    def __init__(self):
        """Initializes the GPSDataParser class."""
        pass
    
    def parse_gagsv(self, nmea_sentence: str) -> list[int] | str | None:
        """Extracts SNR values from a GAGSV or GBGSV sentence."""
        if nmea_sentence == "":
            return "NO CONNECTION!!!"
        fields = nmea_sentence.split(',')
        if not fields[0].startswith("GAGSV") and not fields[0].startswith("GBGSV"):
            return None
        snr_values = []
        for i in range(4, len(fields) - 1, 4):
            if i + 3 < len(fields):
                snr = fields[i + 3]
                if snr.isdigit():
                    snr_values.append(int(snr))
        return snr_values
    
    def compute_average_snr(self, snr_values: list[int]) -> float:
        """Computes the average of SNR values."""
        if not snr_values:
            return 0
        return sum(snr_values) / len(snr_values)
    
    def parse_gngll(self, nmea_sentence: str) -> str | None:
        """Formats latitude and longitude from a GNGLL sentence."""
        if nmea_sentence.strip() == "":
            return "NO CONNECTION!!!"
        fields = nmea_sentence.split(',')
        if not fields[0].startswith("GNGLL"):
            return None
        lat = fields[1]
        lat_dir = fields[2]
        lon = fields[3]
        lon_dir = fields[4]
        return f'Latitude: {lat} {lat_dir} Longitude: {lon} {lon_dir}'
    
    def parse_gngll_to_decimal(self, nmea_sentence: str) -> Optional[Coordinate]:
        """
        Parses GNGLL sentence and returns decimal degree coordinates.
        
        Args:
            nmea_sentence: NMEA GNGLL sentence
            
        Returns:
            Coordinate object or None if invalid
        """
        if nmea_sentence.strip() == "":
            return None
        fields = nmea_sentence.split(',')
        if not fields[0].startswith("GNGLL") or len(fields) < 5:
            return None
        
        try:
            # Parse latitude (format: DDMM.MMMM)
            lat_raw = fields[1]
            lat_dir = fields[2]
            lat_deg = int(lat_raw[:2])
            lat_min = float(lat_raw[2:])
            lat_decimal = lat_deg + (lat_min / 60.0)
            if lat_dir == 'S':
                lat_decimal = -lat_decimal
            
            # Parse longitude (format: DDDMM.MMMM)
            lon_raw = fields[3]
            lon_dir = fields[4]
            lon_deg = int(lon_raw[:3])
            lon_min = float(lon_raw[3:])
            lon_decimal = lon_deg + (lon_min / 60.0)
            if lon_dir == 'W':
                lon_decimal = -lon_decimal
            
            return Coordinate(lat_decimal, lon_decimal)
        except (ValueError, IndexError):
            return None
    
    def parse_message(self, nmea_sentence: str) -> tuple[list[int] | str | None, float, str | None]:
        """Parses an NMEA sentence and returns SNR values, average SNR, and coordinates."""
        snr_values = self.parse_gagsv(nmea_sentence)
        average_snr = 0
        coordinates = None
        
        if snr_values is None:
            coordinates = self.parse_gngll(nmea_sentence)
        elif isinstance(snr_values, list):
            average_snr = self.compute_average_snr(snr_values)
        
        return snr_values, average_snr, coordinates


class AutonomousNavigator:
    """
    Handles autonomous navigation for a rover using GPS coordinates.
    Calculates bearing, distance, and provides navigation commands.
    """
    
    def __init__(self, arrival_threshold: float = 2.0, min_snr: float = 20.0):
        """
        Initialize the autonomous navigator.
        
        Args:
            arrival_threshold: Distance in meters to consider arrived at waypoint
            min_snr: Minimum SNR value to trust GPS data
        """
        self.gps_parser = GPSDataParser()
        self.current_position: Optional[Coordinate] = None
        self.target_waypoint: Optional[Coordinate] = None
        self.waypoint_queue: List[Coordinate] = []
        self.arrival_threshold = arrival_threshold
        self.min_snr = min_snr
        self.status = NavigationStatus.IDLE
        
    def haversine_distance(self, coord1: Coordinate, coord2: Coordinate) -> float:
        """
        Calculate distance between two coordinates using Haversine formula.
        
        Args:
            coord1: Starting coordinate
            coord2: Ending coordinate
            
        Returns:
            Distance in meters
        """
        R = 6371000  # Earth's radius in meters
        
        lat1, lon1 = math.radians(coord1.latitude), math.radians(coord1.longitude)
        lat2, lon2 = math.radians(coord2.latitude), math.radians(coord2.longitude)
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        
        return R * c
    
    def calculate_bearing(self, start: Coordinate, end: Coordinate) -> float:
        """
        Calculate bearing from start to end coordinate.
        
        Args:
            start: Starting coordinate
            end: Ending coordinate
            
        Returns:
            Bearing in degrees (0-360, where 0 is North)
        """
        lat1, lon1 = math.radians(start.latitude), math.radians(start.longitude)
        lat2, lon2 = math.radians(end.latitude), math.radians(end.longitude)
        
        dlon = lon2 - lon1
        
        x = math.sin(dlon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
        
        bearing = math.atan2(x, y)
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        
        return bearing
    
    def update_position(self, nmea_sentence: str, current_snr: float) -> bool:
        """
        Update current position from NMEA sentence.
        
        Args:
            nmea_sentence: GNGLL NMEA sentence
            current_snr: Current average SNR value
            
        Returns:
            True if position updated successfully
        """
        if current_snr < self.min_snr:
            self.status = NavigationStatus.NO_GPS
            return False
        
        coord = self.gps_parser.parse_gngll_to_decimal(nmea_sentence)
        if coord:
            self.current_position = coord
            return True
        return False
    
    def set_waypoint(self, waypoint: Coordinate):
        """Set a single target waypoint."""
        self.target_waypoint = waypoint
        self.status = NavigationStatus.NAVIGATING
    
    def add_waypoint(self, waypoint: Coordinate):
        """Add waypoint to queue."""
        self.waypoint_queue.append(waypoint)
        if self.status == NavigationStatus.IDLE:
            self.next_waypoint()
    
    def next_waypoint(self):
        """Move to next waypoint in queue."""
        if self.waypoint_queue:
            self.target_waypoint = self.waypoint_queue.pop(0)
            self.status = NavigationStatus.NAVIGATING
        else:
            self.target_waypoint = None
            self.status = NavigationStatus.IDLE
    
    def get_navigation_command(self, rover_heading: float) -> dict:
        """
        Generate navigation command for rover.
        
        Args:
            rover_heading: Current rover heading in degrees (0-360)
            
        Returns:
            Dictionary with navigation instructions:
            {
                'status': NavigationStatus,
                'distance': float (meters),
                'bearing': float (degrees),
                'turn_angle': float (degrees, + = right, - = left),
                'action': str ('forward', 'turn_left', 'turn_right', 'arrived', 'wait')
            }
        """
        if self.current_position is None:
            return {
                'status': NavigationStatus.NO_GPS,
                'distance': 0,
                'bearing': 0,
                'turn_angle': 0,
                'action': 'wait'
            }
        
        if self.target_waypoint is None:
            return {
                'status': NavigationStatus.IDLE,
                'distance': 0,
                'bearing': 0,
                'turn_angle': 0,
                'action': 'wait'
            }
        
        distance = self.haversine_distance(self.current_position, self.target_waypoint)
        
        # Check if arrived
        if distance <= self.arrival_threshold:
            self.status = NavigationStatus.ARRIVED
            self.next_waypoint()  # Move to next waypoint if available
            return {
                'status': NavigationStatus.ARRIVED,
                'distance': distance,
                'bearing': 0,
                'turn_angle': 0,
                'action': 'arrived'
            }
        
        # Calculate bearing to target
        target_bearing = self.calculate_bearing(self.current_position, self.target_waypoint)
        
        # Calculate turn angle needed
        turn_angle = target_bearing - rover_heading
        # Normalize to -180 to 180
        turn_angle = (turn_angle + 180) % 360 - 180
        
        # Determine action
        if abs(turn_angle) > 15:  # Need to turn if off by more than 15 degrees
            action = 'turn_right' if turn_angle > 0 else 'turn_left'
        else:
            action = 'forward'
        
        return {
            'status': NavigationStatus.NAVIGATING,
            'distance': distance,
            'bearing': target_bearing,
            'turn_angle': turn_angle,
            'action': action
        }


# Example usage
if __name__ == "__main__":
    # Initialize navigator
    navigator = AutonomousNavigator(arrival_threshold=2.0, min_snr=20.0)
    
    # Example NMEA sentence (GNGLL format)
    # Format: $GNGLL,DDMM.MMMM,N/S,DDDMM.MMMM,E/W,...
    gps_sentence = "$GNGLL,4807.038,N,01131.000,E,123519,A,*7C"
    
    # Update position
    current_snr = 35.0  # Good signal
    if navigator.update_position(gps_sentence, current_snr):
        print(f"Current position: {navigator.current_position}")
    
    # Set target waypoint (slightly north)
    target = Coordinate(48.1180, 11.5167)  # Example: ~100m away
    navigator.set_waypoint(target)
    
    # Get navigation command (assuming rover facing north = 0°)
    rover_heading = 0.0
    command = navigator.get_navigation_command(rover_heading)
    
    print(f"\nNavigation Command:")
    print(f"  Status: {command['status'].value}")
    print(f"  Distance to target: {command['distance']:.2f} meters")
    print(f"  Target bearing: {command['bearing']:.1f}°")
    print(f"  Turn angle needed: {command['turn_angle']:.1f}°")
    print(f"  Action: {command['action']}")