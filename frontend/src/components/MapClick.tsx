import { ClickStatusOptions, MapData } from '../types';
import { useMapEvents } from 'react-leaflet';
import { addMapData } from '../utils/helper';

type MapClickProps = {
  clickStatus: ClickStatusOptions;
  dispatch: any;
};

export default function MapClick({ clickStatus, dispatch }: MapClickProps) {
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
  });
  return <></>;
}
