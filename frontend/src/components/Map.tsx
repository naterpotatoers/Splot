import { MapContainer, Polyline, Rectangle, TileLayer } from 'react-leaflet';
import MapClick from './MapClick';
import MapMarker from './MapMarker';
import { MapReducerState } from '../reducer/mapReducer';
import { ClickStatus, MapData } from '../types';
import { InterestMarker, WaypointMarker } from '../utils/marker-icons';
import { removeScoutWaypoint } from '../api/waypoint';
import { removeMarker } from '../api/marker';

type MapProps = {
  clickStatus: ClickStatus;
  dispatch: any;
  mapData: MapReducerState;
};

export default function Map({ clickStatus, dispatch, mapData }: MapProps) {
  const handleRemoveMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_removed', payload: mapData });
    removeMarker(mapData);
  };

  const handleRemoveWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_removed', payload: mapData });
    removeScoutWaypoint(mapData);
  };

  const mapCenter = { lat: 0, lng: 0 };
  const perimeterCoordinates = [
    [0, 0],
    [0, 0],
  ];
  const exploredCoordinates = [
    [0, 0],
    [0, 0],
  ];
  return (
    <MapContainer
      style={{ height: '100%', width: '100%', minHeight: '400px' }}
      center={mapCenter}
      id="splot-map"
      zoom={7}
      scrollWheelZoom={false}
      doubleClickZoom={false}
    >
      <TileLayer
        attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a>'
        url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
      />

      <Polyline
        pathOptions={{ color: 'black' }}
        positions={exploredCoordinates}
        weight={10}
        opacity={0.5}
      />

      <Rectangle
        bounds={perimeterCoordinates}
        color="black"
        fillOpacity={0.05}
      />

      <MapClick clickStatus={clickStatus} dispatch={dispatch} />

      <MapMarker
        label="Interest"
        mapData={mapData.marker}
        remove={handleRemoveMarker}
        icon={InterestMarker}
      />

      <MapMarker
        label="Waypoint"
        mapData={mapData.waypoint}
        remove={handleRemoveWaypoint}
        icon={WaypointMarker}
      />
    </MapContainer>
  );
}
