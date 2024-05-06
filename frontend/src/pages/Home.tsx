import { useEffect, useReducer, useState } from 'react';
import { DEFAULT_MAP_DATA } from '../utils/constants';
import { ClickStatus, MapData } from '../types';
import { InterestMarker, WaypointMarker } from '../utils/marker-icons';
import { MapContainer, Polyline, Rectangle, TileLayer } from 'react-leaflet';
import {
  getMapCenter,
  getPerimeterCoordinates,
  groupCoordinatesById,
} from '../utils';
import { removeScoutWaypoint } from '../api/waypoint';
import { removeMarker } from '../api/marker';
import { getAllMapPositions } from '../utils/helper';
import mapReducer from '../reducer/mapReducer';
import MapMarker from '../components/MapMarker';
import MapClick from '../components/MapClick';
import Header from '../components/Header';
import MapForm from '../components/MapForm';

export default function Home() {
  const [clickStatus, setClickStatus] = useState<ClickStatus>('perimeter');
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  useEffect(() => {
    const interval = setInterval(() => {
      const fetchData = async () => {
        const response = await getAllMapPositions();
        dispatch({ type: 'set_all', payload: response });
      };
      fetchData();
    }, 1000);
    return () => clearInterval(interval);
  }, []);

  const handleRemoveMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_removed', payload: mapData });
    removeMarker(mapData);
  };

  const handleRemoveWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_removed', payload: mapData });
    removeScoutWaypoint(mapData);
  };

  const perimeterCoordinates = getPerimeterCoordinates(mapData.perimeter);
  const exploredCoordinates = groupCoordinatesById(mapData.explored);

  return (
    <div>
      <Header clickStatus={clickStatus} setClickStatus={setClickStatus} />
      <MapContainer
        style={{ height: '100%', width: '100%', minHeight: '75vh' }}
        center={getMapCenter(mapData.perimeter)}
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
          label="markers"
          mapData={mapData.marker}
          remove={handleRemoveMarker}
          icon={InterestMarker}
        />

        <MapMarker
          label="waypoints"
          mapData={mapData.waypoint}
          remove={handleRemoveWaypoint}
          icon={WaypointMarker}
        />
      </MapContainer>
      <MapForm mapData={mapData} dispatch={dispatch} />
    </div>
  );
}
