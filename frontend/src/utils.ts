import { MapCoordinate, MapMarker } from './types';

export function getMapCenter(markers: Array<MapMarker>): MapCoordinate {
  if (markers.length === 0) return { lat: 0, lng: 0 };
  const lat =
    markers.reduce((acc, marker) => acc + marker.coords.lat, 0) /
    markers.length;
  const lng =
    markers.reduce((acc, marker) => acc + marker.coords.lng, 0) /
    markers.length;
  return {
    lat: parseFloat(lat.toFixed(2)),
    lng: parseFloat(lng.toFixed(2)),
  };
}
