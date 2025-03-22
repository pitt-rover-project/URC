class SNR:
    def parse_gagsv(nmea_sentence):
        if nmea_sentence == "":
            return "NO CONNECTION!!!"
        
        fields = nmea_sentence.split(',')
        
        # Check if it's a valid GAGSV message
        if not fields[0].startswith("GAGSV") or fields[0].startswith("GBGSV"):
            return None  # Not a GAGSV or GBGSV sentence
        
        snr_values = []  # Array to store extracted SNR values
        
        # Satellite data starts at index 4 and repeats every 4 elements
        for i in range(4, len(fields) - 1, 4):
            if i + 3 < len(fields):  # Ensure SNR field exists
                snr = fields[i + 3]  # SNR is the 4th value in the set
                if snr.isdigit():  # Some SNR values might be missing (empty)
                    snr_values.append(int(snr))
        
        return snr_values
    
    # Example
    gagsv_sentence = "GAGSV,2,1,05,02,74,123,46,25,21,129,39,30,48,312,24,34,56,204,26,278"
    snr_list = parse_gagsv(gagsv_sentence)
    print(snr_list)  # Output: [46, 39, 24, 26]

    sum = 0
    count = 0
    for i in snr_list: # Compute average SNR, higher is better
        sum += i
        count += 1

    average = sum / count
    print ("Average SNR: ", average)

class Coordinates:
    def parse_gngll(nmea_sentence):
        if nmea_sentence == "":
            return "NO CONNECTION!!!"
        
        fields = nmea_sentence.split(',')
        
        # Check if it's a valid GNGLL message
        if not fields[0].startswith("GNGLL"):
            return None  # Not a GNGLL sentence
        
        lat = fields[1]
        lat_dir = fields[2]
        lon = fields[3]
        lon_dir = fields[4]

        return 'Latitude: ' + lat + ' ' + lat_dir + ' Longitude: ' + lon + ' ' + lon_dir

    # Example
    gngll_sentence = "GNGLL,4026.62593,N,07957.49971,W,210345.00,A,A*61"
    coordinates = parse_gngll(gngll_sentence)
    print (coordinates)