import { MOCK_MAP_DATA } from '../../utils/constants';
import mapReducer from '../mapReducer';

describe('Map Reducer', () => {
  it('should add a marker', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'marker_added',
      payload: {
        id: 'test-marker-1',
        desc: 'test marker 1',
        coords: { lat: 36.77, lng: -119.74 },
      },
    };
    const result = mapReducer(state, action);
    expect(result.marker.length).greaterThan(MOCK_MAP_DATA.marker.length);
  });

  it('should remove a marker', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'marker_removed',
      payload: MOCK_MAP_DATA.marker[0],
    };
    const result = mapReducer(state, action);
    expect(result.marker.length).lessThan(MOCK_MAP_DATA.marker.length);
  });

  it('should add a perimeter', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'perimeter_added',
      payload: {
        id: 'test-perimeter-1',
        desc: 'test perimeter 1',
        coords: { lat: 36.77, lng: -119.74 },
      },
    };
    const result = mapReducer(state, action);
    expect(result.perimeter.length).greaterThan(MOCK_MAP_DATA.perimeter.length);
  });

  it('should remove a perimeter', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'perimeter_removed',
      payload: MOCK_MAP_DATA.perimeter[0],
    };
    const result = mapReducer(state, action);
    expect(result.perimeter.length).lessThan(MOCK_MAP_DATA.perimeter.length);
  });

  it('should add an explored', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'explored_added',
      payload: {
        id: 'test-explored-1',
        desc: 'test explored 1',
        coords: { lat: 36.77, lng: -119.74 },
      },
    };
    const result = mapReducer(state, action);
    expect(result.explored.length).greaterThan(MOCK_MAP_DATA.explored.length);
  });

  it('should remove an explored', () => {
    const state = MOCK_MAP_DATA;
    const action = {
      type: 'explored_removed',
      payload: MOCK_MAP_DATA.explored[0],
    };
    const result = mapReducer(state, action);
    expect(result.explored.length).lessThan(MOCK_MAP_DATA.explored.length);
  });
});
