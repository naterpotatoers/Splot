import { MapReducerState } from './reducer/mapReducer';
import { ClickStatusOptions, MapCoordinate, MapData } from './types';

export function handleMapInputSubmit(
  e: React.FormEvent<HTMLFormElement>,
  label: string,
): MapData {
  e.preventDefault();
  const form = e.currentTarget;
  const lat = parseFloat(form.lat.value);
  const lng = parseFloat(form.lng.value);
  const desc = form.desc.value;
  const newMarker: MapData = {
    id: `manually-set-${label.toLowerCase()}-` + Date.now().toString(),
    coords: { lat, lng },
    desc,
  };
  form.reset();
  return newMarker;
}

export function getMapCenter(perimeter: Array<MapData>): MapCoordinate {
  if (perimeter.length === 0) return { lat: 0, lng: 0 };
  const lat =
    perimeter.reduce((acc, marker) => acc + marker.coords.lat, 0) /
    perimeter.length;
  const lng =
    perimeter.reduce((acc, marker) => acc + marker.coords.lng, 0) /
    perimeter.length;
  return {
    lat: parseFloat(lat.toFixed(2)),
    lng: parseFloat(lng.toFixed(2)),
  };
}

export function getPerimeterCoordinates(
  perimeter: Array<MapData>,
): Array<MapCoordinate> {
  if (perimeter.length === 0) return [{ lat: 0, lng: 0 }];
  return perimeter.map((marker) => marker.coords);
}

export function getUniqueDeviceId(data: Array<MapData>): Array<string> {
  // grab everything before the second hyphen
  const ids = data.map((marker) => marker.id.split('-').slice(0, 2).join('-'));
  return Array.from(new Set(ids));
}

export function groupCoordinatesById(
  explored: Array<MapData>,
): Array<Array<MapCoordinate>> {
  const ids = getUniqueDeviceId(explored);
  const grouped = ids.map((id) =>
    explored.filter((marker) => marker.id.startsWith(id)),
  );
  return grouped.map((group) => group.map((marker) => marker.coords));
}
