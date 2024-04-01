import { ClickStatusOptions, MapData } from '../types';
import { useMapEvents } from 'react-leaflet';

type MapClickProps = {
  clickStatus: ClickStatusOptions;
  dispatch: any;
};

export default function MapClick({ clickStatus, dispatch }: MapClickProps) {
  useMapEvents({
    dblclick(e) {
      const newMapData: MapData = {
        id: `manually-set-${clickStatus}-` + Date.now().toString(),
        coords: e.latlng,
        desc: `${clickStatus}`,
      };
      dispatch({ type: `${clickStatus}_added`, payload: newMapData });
    },
  });
  return <></>;
}
