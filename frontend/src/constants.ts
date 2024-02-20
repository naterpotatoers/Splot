import { MapData, MapMarker } from './types';

export const MAP_MARKERS: Array<MapMarker> = [
  {
    id: 'manually-set-marker-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: 'First marker desc',
  },
  {
    id: 'manually-set-marker-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: 'Second marker desc',
  },
  {
    id: 'manually-set-marker-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Third marker desc',
  },
];

export const MAP_PERIMETER: Array<MapMarker> = [
  {
    id: 'manually-set-perimeter-1',
    coords: { lat: 36.77, lng: -119.74 },
    desc: 'First perimeter coordinate desc',
  },
  {
    id: 'manually-set-perimeter-2',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Second perimeter coordinate desc',
  },
];

export const MAP_EXPLORED: Array<MapMarker> = [
  {
    id: 'explored-drone-x-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: 'First explored coordinate by drone x',
  },
  {
    id: 'explored-drone-x-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: 'Second explored coordinate by drone x',
  },
  {
    id: 'explored-drone-x-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Third explored coordinate by drone x',
  },
  {
    id: 'explored-drone-y-1',
    coords: { lat: 36.74, lng: -119.72 },
    desc: 'First explored coordinate by drone y',
  },
  {
    id: 'explored-drone-y-2',
    coords: { lat: 36.73, lng:-119.73 },
    desc: 'Second explored coordinate by drone y',
  },
  {
    id: 'explored-drone-y-3',
    coords: { lat: 36.72, lng: -119.74 },
    desc: 'Third explored coordinate by drone y',
  },
  {
    id: 'explored-drone-z-1',
    coords: { lat: 36.77, lng: -119.73 },
    desc: 'First explored coordinate by drone z',
  },
  {
    id: 'explored-drone-z-2',
    coords: { lat: 36.74, lng: -119.72 },
    desc: 'Second explored coordinate by drone z',
  },
  {
    id: 'explored-drone-z-3',
    coords: { lat: 36.71, lng: -119.71 },
    desc: 'Third explored coordinate by drone z',
  },
];


export const DEFAULT_MAP_DATA: MapData = {
  markers: MAP_MARKERS,
  perimeter: MAP_PERIMETER,
  explored: MAP_EXPLORED,
};
