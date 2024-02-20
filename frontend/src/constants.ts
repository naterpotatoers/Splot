import { MapData, MapMarker } from './types';

export const MAP_MARKERS: Array<MapMarker> = [
  {
    id: 'id-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: 'First marker desc',
  },
  {
    id: 'id-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: 'Second marker desc',
  },
  {
    id: 'id-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Third marker desc',
  },
];

export const MAP_PERIMETER: Array<MapMarker> = [
  {
    id: 'id-4',
    coords: { lat: 36.77, lng: -119.74 },
    desc: 'First perimeter coordinate desc',
  },
  {
    id: 'id-5',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Second perimeter coordinate desc',
  },
];

export const DEFAULT_MAP_DATA: MapData = {
  markers: MAP_MARKERS,
  perimeter: MAP_PERIMETER,
  explored: [],
};
