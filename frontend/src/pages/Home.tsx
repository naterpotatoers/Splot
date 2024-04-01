import { useReducer, useState } from 'react';
import { DEFAULT_MAP_DATA } from '../utils/constants';
import { ClickStatusOptions, MapData } from '../types';
import { MapContainer, Polyline, Rectangle, TileLayer } from 'react-leaflet';
import {
  getMapCenter,
  getPerimeterCoordinates,
  groupCoordinatesById,
} from '../utils';

import mapReducer from '../reducer/mapReducer';
import MapMarker from '../components/MapMarker';
import MapInput from '../components/MapInput';
import MapList from '../components/MapList';
import Header from '../components/Header';
import MapClick from '../components/MapClick';

export default function Home() {
  const [clickStatus, setClickStatus] =
    useState<ClickStatusOptions>('perimeter');
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  const handleAddMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_added', payload: mapData });
  };

  const handleRemoveMarker = (mapData: MapData) => {
    dispatch({ type: 'marker_removed', payload: mapData });
  };

  const handleAddPerimeter = (mapData: MapData) => {
    dispatch({ type: 'perimeter_added', payload: mapData });
  };

  const handleRemovePerimeter = (mapData: MapData) => {
    dispatch({ type: 'perimeter_removed', payload: mapData });
  };

  const handleAddExplored = (mapData: MapData) => {
    dispatch({ type: 'explored_added', payload: mapData });
  };

  const handleRemoveExplored = (mapData: MapData) => {
    dispatch({ type: 'explored_removed', payload: mapData });
  };

  const handleAddWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_added', payload: mapData });
  };

  const handleRemoveWaypoint = (mapData: MapData) => {
    dispatch({ type: 'waypoint_removed', payload: mapData });
  };

  const perimeterCoordinates = getPerimeterCoordinates(mapData.perimeter);
  const exploredCoordinates = groupCoordinatesById(mapData.explored);

  return (
    <div>
      <Header clickStatus={clickStatus} setClickStatus={setClickStatus} />
      <MapContainer
        style={{ height: '100%', width: '100%', minHeight: '400px' }}
        center={getMapCenter(mapData.perimeter)}
        id="splot-map"
        zoom={12}
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
        />

        <MapMarker
          label="Waypoint"
          mapData={mapData.waypoint}
          remove={handleRemoveWaypoint}
        />
      </MapContainer>

      <div className="grid-col-3 gap">
        <div id="map-marker-data">
          <MapInput label="Interest Marker" create={handleAddMarker} />
          <MapList
            label="Interest Marker"
            list={mapData.marker}
            remove={handleRemoveMarker}
          />
        </div>
        <div id="map-waypoint-data">
          <MapInput label="Waypoint" create={handleAddWaypoint} />
          <MapList
            label="Waypoint"
            list={mapData.waypoint}
            remove={handleRemoveWaypoint}
          />
        </div>
        <div id="map-perimeter-data">
          <MapInput label="Perimeter" create={handleAddPerimeter} />
          <MapList
            label="Perimeter"
            list={mapData.perimeter}
            remove={handleRemovePerimeter}
          />
        </div>
        <div id="map-explored-data">
          <MapInput label="Explored" create={handleAddExplored} />
          <MapList
            label="Explored"
            list={mapData.explored}
            remove={handleRemoveExplored}
          />
        </div>
      </div>
    </div>
  );
}
