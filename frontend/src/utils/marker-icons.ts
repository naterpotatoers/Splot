import { Icon } from 'leaflet';
import interestMarker from '../assets/map-marker-star.svg';
import waypoint from '../assets/blue-map-marker.svg';

export const InterestMarker = new Icon({
  iconUrl: interestMarker,
  iconSize: [40, 50],
  iconAnchor: [20, 38],
  popupAnchor: [1, -34],
});

export const WaypointMarker = new Icon({
  iconUrl: waypoint,
  iconSize: [30, 31],
  iconAnchor: [15, 28],
  popupAnchor: [1, -34],
});
