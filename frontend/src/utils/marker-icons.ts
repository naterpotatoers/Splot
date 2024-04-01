import { Icon } from 'leaflet';
import starMarker from '../assets/map-marker-star.svg';

export const InterestMarker = new Icon({
  iconUrl: starMarker,
  iconSize: [40, 51],
  iconAnchor: [20, 39],
  popupAnchor: [1, -34],
});
