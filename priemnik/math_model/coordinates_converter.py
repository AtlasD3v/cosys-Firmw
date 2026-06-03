import numpy as np
class Coord_converter:
    def __init__(self):
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        # Экваториальный радиус Земли (WGS-84)
        self.R_EARTH = 6378137.0
        self.Pi = 3.1415926535

    def initialize_home_coordinates(self, lat, lon, alt):
        self.home_lat = lat
        self.home_lon = lon
        self.home_alt = alt

    
    def convert_coords_to_local_meters(self, new_lat, new_lon, new_alt):
        #Перевод координат Lat/Lon/Alt -> Локальные метры (NED)
        # Переводим дельты углов в радианы
        new_lat_rad = new_lat * (self.Pi / 180.0)
        home_lat_rad = self.home_lat * (self.Pi / 180.0)
        new_pos_x = (new_lat_rad - home_lat_rad) * self.R_EARTH #получаем дистанцию, которую мы пролетели он начальной точки

        new_lon_rad = new_lon * (self.Pi / 180.0)
        home_lon_rad = self.home_lon * (self.Pi / 180.0)
        new_pos_y = (new_lon_rad - home_lon_rad) * self.R_EARTH * np.cos(home_lat_rad)

        new_pos_z = self.home_alt - new_alt

        return [new_pos_x, new_pos_y, new_pos_z]