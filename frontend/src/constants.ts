import { MapData, MapMarker } from './types';

export const MAP_MARKERS: Array<MapMarker> = [
  {
    id: 'manually-set-marker-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: '1st marker',
  },
  {
    id: 'manually-set-marker-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: '2nd marker',
  },
  {
    id: 'manually-set-marker-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: '3rd marker',
  },
];

export const MAP_PERIMETER: Array<MapMarker> = [
  {
    id: 'manually-set-perimeter-1',
    coords: { lat: 36.77, lng: -119.74 },
    desc: '1st perimeter',
  },
  {
    id: 'manually-set-perimeter-2',
    coords: { lat: 36.71, lng: -119.71 },
    desc: '2nd perimeter',
  },
];

export const MAP_EXPLORED: Array<MapMarker> = [
  {
    id: 'drone-x-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: '1st explored',
  },
  {
    id: 'drone-x-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: '2nd explored',
  },
  {
    id: 'drone-x-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: '3rd explored',
  },
  {
    id: 'drone-y-1',
    coords: { lat: 36.74, lng: -119.72 },
    desc: '1st explored',
  },
  {
    id: 'drone-y-2',
    coords: { lat: 36.73, lng:-119.73 },
    desc: '2nd explored',
  },
  {
    id: 'drone-y-3',
    coords: { lat: 36.72, lng: -119.74 },
    desc: '3rd explored',
  },
  {
    id: 'drone-z-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: '1st explored',
  },
  {
    id: 'drone-z-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: '2nd explored',
  },
  {
    id: 'drone-z-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: '3rd explored',
  },
];


export const DEFAULT_MAP_DATA: MapData = {
  markers: MAP_MARKERS,
  perimeter: MAP_PERIMETER,
  explored: MAP_EXPLORED,
};
