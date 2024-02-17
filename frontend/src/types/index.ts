export type GpsCoordinate = {
  latitude: number;
  longitude: number;
};

export type GpsMarker = {
  id: string;
  gpsCoordinate: GpsCoordinate;
  description: string;
};
