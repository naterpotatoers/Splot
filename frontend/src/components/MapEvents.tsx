import { ClickStatus, MapData } from '../types';
import { useMapEvents } from 'react-leaflet';
import { addMapData } from '../utils/helper';

type MapClickProps = {
  dispatch: any;
  clickStatus: ClickStatus;
};

export default function MapEvents({ clickStatus, dispatch }: MapClickProps) {
  useMapEvents({
    async dblclick(e) {
      const newMapData: MapData = {
        id: `manually-set-${clickStatus}-` + Date.now().toString(),
        coords: e.latlng,
        desc: `${clickStatus}`,
      };
      dispatch({ type: `${clickStatus}_added`, payload: newMapData });
      addMapData(clickStatus, newMapData);
    },
    async moveend(e) {
      console.log('Map moved', e.target.getCenter());
    },
    async zoomstart(e) {
      console.log('Map zooming', e.target.getZoom());
    },
    locationfound(e) {
      console.log('Location found', e.latlng);
    }
  });
  return null;
}
