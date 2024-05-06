import MapMarker from './MapMarker';
import MapEvents from './MapEvents';
import { removeMarker } from '../api/marker';
import { ClickStatus, MapData } from '../types';
import { removeScoutWaypoint } from '../api/waypoint';
import { MapReducerState } from '../reducer/mapReducer';
import { InterestMarker, WaypointMarker } from '../utils/marker-icons';
import {
  getMapCenter,
  getPerimeterCoordinates,
  groupCoordinatesById,
} from '../utils';
import { MapContainer, Polyline, Rectangle, TileLayer } from 'react-leaflet';
import { useCallback, useState } from 'react';

type MapProps = {
  dispatch: any;
  clickStatus: ClickStatus;
  mapData: MapReducerState;
};

export default function Map({ clickStatus, dispatch, mapData }: MapProps) {
  const [map, setMap] = useState(null);
  const handleRemoveMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_removed', payload: mapData });
    removeMarker(mapData);
  };

  const handleMapFlyTo = useCallback(() => {
    if (map) {
      (map as any).flyTo(getMapCenter(mapData.perimeter), 7);
    }
  }, [map, mapData.perimeter]);
  handleMapFlyTo();

  const handleRemoveWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_removed', payload: mapData });
    removeScoutWaypoint(mapData);
  };

  const mapCenter = getMapCenter(mapData.perimeter);
  const perimeterCoordinates = getPerimeterCoordinates(mapData.perimeter);
  const exploredCoordinates = groupCoordinatesById(mapData.explored);

  return (
    <MapContainer
      style={{ minHeight: '80vh' }}
      center={mapCenter}
      ref={setMap}
      id="splot-map"
      zoom={7}
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

      <MapEvents clickStatus={clickStatus} dispatch={dispatch} />

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
