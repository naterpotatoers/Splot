export type MapCoordinate = {
  lat: number;
  lng: number;
};

export type MapMarker = {
  id: string;
  desc: string;
  coords: MapCoordinate;
};
