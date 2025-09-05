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
        """
        Extracts SNR values from a GAGSV or GBGSV sentence.

        Args:
            nmea_sentence (str): NMEA sentence.

        Returns:
            list[int]: Extracted SNR values.
            str: "NO CONNECTION!!!" if empty.
            None: If invalid sentence.
        """
        if nmea_sentence == "":
            return "NO CONNECTION!!!"
        
        fields = nmea_sentence.split(',')
        
        # Check if it's a valid GAGSV or GBGSV message
        if not fields[0].startswith("GAGSV") and not fields[0].startswith("GBGSV"):
            return None  # Not a GAGSV or GBGSV sentence
        
        snr_values = []
        
        # Satellite data starts at index 4 and repeats every 4 elements
        for i in range(4, len(fields) - 1, 4):
            if i + 3 < len(fields):  # Ensure SNR field exists
                snr = fields[i + 3]  # SNR is the 4th value in the set
                if snr.isdigit():  # Some SNR values might be missing (empty)
                    snr_values.append(int(snr))
        return snr_values

    def compute_average_snr(self, snr_values: list[int]) -> float:
        """
        Computes the average of SNR values.

        Args:
            snr_values (list[int]): SNR values.

        Returns:
            float: Average SNR or 0 if the list is empty.
        """
        if not snr_values:
            return 0
        return sum(snr_values) / len(snr_values)

    def parse_gngll(self, nmea_sentence: str) -> str | None:
        """
        Formats latitude and longitude from a GNGLL sentence.

        Args:
            nmea_sentence (str): NMEA sentence.

        Returns:
            str: Formatted latitude and longitude.
            str: "NO CONNECTION!!!" if empty.
            None: If invalid sentence.
        """
        if nmea_sentence.strip() == "":
            return "NO CONNECTION!!!"
        
        fields = nmea_sentence.split(',')
        
        # Check if it's a valid GNGLL message
        if not fields[0].startswith("GNGLL"):
            return None
        
        lat = fields[1]
        lat_dir = fields[2]
        lon = fields[3]
        lon_dir = fields[4]

        return f'Latitude: {lat} {lat_dir} Longitude: {lon} {lon_dir}'

    def parse_message(self, nmea_sentence: str) -> tuple[list[int] | str | None, float, str | None]:
        """
        Parses an NMEA sentence and returns the list of extracted SNR values,
        the average SNR value, and the coordinates (if any).

        Args:
            nmea_sentence (str): The NMEA sentence to parse.

        Returns:
            tuple: (snr_values, average_snr, coordinates)
                   - snr_values: list of SNR integers, "NO CONNECTION!!!", or None
                   - average_snr: float average of SNR or 0 if none
                   - coordinates: string of coordinates, "NO CONNECTION!!!", or None
        """
        # Try parsing SNR first (GAGSV/GBGSV)
        snr_values = self.parse_gagsv(nmea_sentence)
        average_snr = 0
        coordinates = None

        # If this isn't a valid GAGSV/GBGSV message, try to parse GNGLL
        if snr_values is None:
            coordinates = self.parse_gngll(nmea_sentence)
        # If snr_values is a list of integers, compute average
        elif isinstance(snr_values, list):
            average_snr = self.compute_average_snr(snr_values)

        return snr_values, average_snr, coordinates