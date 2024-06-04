import { useMapEvents } from 'react-leaflet'

export default function MapCenterEvent() {
    useMapEvents({
        async moveend(e) {
            console.log('Map moved', e.target.getCenter())
        },
        async zoomstart(e) {
            console.log('Map zooming', e.target.getZoom())
        }
    })
  return null
}
