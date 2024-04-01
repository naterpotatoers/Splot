export type MapCoordinate = {
    lat: number;
    lng: number;
  };
  
  export type MapData = {
    id: string;
    desc: string;
    coords: MapCoordinate;
  };
  
  export type MapProps = {
    markers: Array<MapData>;
    perimeter: Array<MapData>;
    explored: Array<MapData>;
  };
  
  export type ClickStatusOptions =
    | 'perimeter'
    | 'marker'
    | 'explored'
    | 'waypoint';
  