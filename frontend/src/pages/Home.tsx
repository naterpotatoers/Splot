import { useReducer, useState } from 'react';
import { DEFAULT_MAP_DATA } from '../constants';
import { MapData } from '../types';
import { MapContainer, Polyline, Rectangle, TileLayer } from 'react-leaflet';
import {
  getMapCenter,
  getPerimeterCoordinates,
  groupCoordinatesById,
} from '../utils';

import MarkerOnDoubleClick from '../components/MarkerOnDoubleClick';
import mapReducer from '../reducer/mapReducer';
import MapInput from '../components/MapInput';
import MapList from '../components/MapList';

export default function Home() {
  const [mapClickMarker, setMapClickMarker] = useState('perimeter');
  const [mapData, dispatch] = useReducer(mapReducer, DEFAULT_MAP_DATA);

  const handleAddMarker = (marker: MapData) => {
    dispatch({ type: 'marker_added', payload: marker });
  };

  const handleRemoveMarker = (marker: MapData) => {
    dispatch({ type: 'marker_removed', payload: marker });
  };

  const handleAddPerimeter = (perimeter: MapData) => {
    dispatch({ type: 'perimeter_added', payload: perimeter });
  };

  const handleRemovePerimeter = (perimeter: MapData) => {
    dispatch({ type: 'perimeter_removed', payload: perimeter });
  };

  const handleAddExplored = (explored: MapData) => {
    dispatch({ type: 'explored_added', payload: explored });
  };

  const handleRemoveExplored = (explored: MapData) => {
    dispatch({ type: 'explored_removed', payload: explored });
  };

  const perimeterCoordinates = getPerimeterCoordinates(mapData.perimeter);
  const exploredCoordinates = groupCoordinatesById(mapData.explored);

  return (
    <div>
      <div className="flex space-between">
        <h1>Splot</h1>
        <div className="flex">
          <h2>{mapClickMarker}</h2>
          <div>
            <button onClick={() => setMapClickMarker('perimeter')}>
              Set Perimeter
            </button>
            <button onClick={() => setMapClickMarker('marker')}>
              Set Marker
            </button>
            <button onClick={() => setMapClickMarker('waypoint')}>
              Set Waypoint
            </button>
          </div>
        </div>
      </div>
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

        <MarkerOnDoubleClick
          label="Marker"
          mapData={mapData.markers}
          create={
            mapClickMarker === 'perimeter'
              ? handleAddPerimeter
              : handleAddMarker
          }
          remove={
            mapClickMarker === 'perimeter'
              ? handleRemovePerimeter
              : handleRemoveMarker
          }
        />
      </MapContainer>

      <div className="grid-col-3 gap">
        <div id="map-marker-data">
          <MapInput label="Marker" create={handleAddMarker} />
          <MapList
            label="Marker"
            list={mapData.markers}
            remove={handleRemoveMarker}
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
