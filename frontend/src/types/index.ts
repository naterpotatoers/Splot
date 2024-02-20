export type MapCoordinate = {
  lat: number;
  lng: number;
};

export type MapMarker = {
  id: string;
  desc: string;
  coords: MapCoordinate;
};

export type MapData = {
  markers: Array<MapMarker>;
  perimeter: Array<MapMarker>;
  explored: Array<MapMarker>;
};
