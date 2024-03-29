import { ClickStatusOptions } from '../types';

type HeaderProps = {
  clickStatus: ClickStatusOptions;
  setClickStatus: (status: ClickStatusOptions) => void;
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

  return (
    <div className="flex space-between padding-10">
      <h1>Splot</h1>
      <div className="flex">
        <h2>{clickStatus}</h2>
        <div>
          <button onClick={handlePerimeterClick}>Set Perimeter</button>
          <button onClick={handleMarkerClick}>Set Marker</button>
          <button onClick={handleWaypointClick}>Set Waypoint</button>
        </div>
      </div>
    </div>
  );
}
