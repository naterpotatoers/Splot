import { ClickStatusOptions } from '../types';

type HeaderProps = {
  clickStatus: string;
  setClickStatus: (status: ClickStatusOptions) => void;
};

export default function Header({ clickStatus, setClickStatus }: HeaderProps) {
  return (
    <div className="flex space-between">
      <h1>Splot</h1>
      <div className="flex">
        <h2>{clickStatus}</h2>
        <div>
          <button onClick={() => setClickStatus('perimeter')}>
            Set Perimeter
          </button>
          <button onClick={() => setClickStatus('marker')}>Set Marker</button>
          <button onClick={() => setClickStatus('waypoint')}>
            Set Waypoint
          </button>
        </div>
      </div>
    </div>
  );
}
