import { Plus } from 'lucide-react';
import React from 'react';
import { MapMarker } from '../types';

type MapExploredInputProps = {
  handleNew: (newMarker: MapMarker) => void;
};

export default function MapExploredInput({
  handleNew,
}: MapExploredInputProps) {
  const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const form = e.currentTarget;
    const lat = parseFloat(form.lat.value);
    const lng = parseFloat(form.lng.value);
    const desc = form.desc.value;
    const newMarker: MapMarker = {
      id: 'manually-explored-' + Date.now().toString(),
      coords: { lat, lng },
      desc,
    };
    handleNew(newMarker);
    form.reset();
  };

  return (
    <>
      <h3>Add Explored</h3>
      <form className="flex-row" onSubmit={handleSubmit}>
        <div className="form-field">
          <label>Latitude</label>
          <input
            type="number"
            name="lat"
            data-testid="explored-lat-input"
            step={0.01}
            max={90}
            min={-90}
            required
          />
        </div>
        <div className="form-field">
          <label>Longitude</label>
          <input
            type="number"
            name="lng"
            data-testid="explored-lng-input"
            step={0.01}
            max={180}
            min={-180}
            required
          />
        </div>
        <div className="form-field">
          <label>Description</label>
          <input type="text" name="desc" data-testid="explored-desc-input" required />
        </div>
        <button type="submit" data-testid="explored-submit-button">
          <Plus />
        </button>
      </form>
    </>
  );
}
