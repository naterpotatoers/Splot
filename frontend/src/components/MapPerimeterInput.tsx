import { Plus } from 'lucide-react';
import React from 'react';
import { MapMarker } from '../types';

type MapPerimeterInputProps = {
  handleNew: (newMarker: MapMarker) => void;
};

export default function MapPerimeterInput({
  handleNew,
}: MapPerimeterInputProps) {
  const handleSubmit = (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();
    const form = e.currentTarget;
    const lat = parseFloat(form.lat.value);
    const lng = parseFloat(form.lng.value);
    const desc = form.desc.value;
    const newMarker: MapMarker = {
      id: Date.now().toString(),
      coords: { lat, lng },
      desc,
    };
    handleNew(newMarker);
    form.reset();
  };

  return (
    <>
      <h3>Add Perimeter</h3>
      <form className="flex-row" onSubmit={handleSubmit}>
        <div className="form-field">
          <label>Latitude</label>
          <input
            type="number"
            name="lat"
            testdata-id="lat-input"
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
            testdata-id="lng-input"
            step={0.01}
            max={180}
            min={-180}
            required
          />
        </div>
        <div className="form-field">
          <label>Description</label>
          <input type="text" name="desc" testdata-id="desc-input" required />
        </div>
        <button type="submit" testdata-id="submit-marker-input">
          <Plus />
        </button>
      </form>
    </>
  );
}
