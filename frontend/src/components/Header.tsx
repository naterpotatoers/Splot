import { removeAllMarkers } from '../api/marker';
import { ClickStatus } from '../types';

type HeaderProps = {
  clickStatus: ClickStatus;
  setClickStatus: (status: ClickStatus) => void;
};

export default function Header({ clickStatus, setClickStatus }: HeaderProps) {
  const handlePerimeterClick = () => {
    setClickStatus('perimeter');
  };

  const handleMarkerClick = () => {
    setClickStatus('marker');
  };

  const handleWaypointClick = () => {
    setClickStatus('waypoint');
  };

  const handleResetClick = async () => {
    // First confirm the deletion since it might have been an accident
  await removeAllMarkers();

  }

  return (
    <div className="flex space-between padding-10">
      <h1>Splot</h1>
      <button data-testid="reset-button" onClick={handleResetClick}>Reset</button>
      <div className="flex">
        <h2 data-testid="current-click-status">{clickStatus}</h2>
        <div>
          <button data-testid="perimeter-button" onClick={handlePerimeterClick}>
            Set Perimeter
          </button>
          <button data-testid="marker-button" onClick={handleMarkerClick}>
            Set Marker
          </button>
          <button data-testid="waypoint-button" onClick={handleWaypointClick}>
            Set Waypoint
          </button>
        </div>
      </div>
    </div>
  );
}
