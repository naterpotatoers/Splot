import { DEFAULT_MAP_DATA } from "../../constants";
import mapReducer from "../mapReducer";

describe('Map Reducer', () => { 
    it('should add a marker', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'marker_added',
            payload: {
                id: 'test-marker-1',
                desc: 'test marker 1',
                coords: { lat: 36.77, lng: -119.74 },
            }
        }
        const result = mapReducer(state, action)
        expect(result.markers.length).greaterThan(DEFAULT_MAP_DATA.markers.length)
    });

    it('should remove a marker', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'marker_removed',
            payload: DEFAULT_MAP_DATA.markers[0]
        }
        const result = mapReducer(state, action)
        expect(result.markers.length).lessThan(DEFAULT_MAP_DATA.markers.length)
    });

    it('should add a perimeter', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'perimeter_added',
            payload: {
                id: 'test-perimeter-1',
                desc: 'test perimeter 1',
                coords: { lat: 36.77, lng: -119.74 },
            }
        }
        const result = mapReducer(state, action)
        expect(result.perimeter.length).greaterThan(DEFAULT_MAP_DATA.perimeter.length)
    });

    it('should remove a perimeter', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'perimeter_removed',
            payload: DEFAULT_MAP_DATA.perimeter[0]
        }
        const result = mapReducer(state, action)
        expect(result.perimeter.length).lessThan(DEFAULT_MAP_DATA.perimeter.length)
    });

    it('should add an explored', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'explored_added',
            payload: {
                id: 'test-explored-1',
                desc: 'test explored 1',
                coords: { lat: 36.77, lng: -119.74 },
            }
        }
        const result = mapReducer(state, action)
        expect(result.explored.length).greaterThan(DEFAULT_MAP_DATA.explored.length)
    });

    it('should remove an explored', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'explored_removed',
            payload: DEFAULT_MAP_DATA.explored[0]
        }
        const result = mapReducer(state, action)
        expect(result.explored.length).lessThan(DEFAULT_MAP_DATA.explored.length)
    });

    it('should handle an unkown action', () => {
        const state = DEFAULT_MAP_DATA;
        const action = {
            type: 'unknown',
            payload: DEFAULT_MAP_DATA.explored[0]
        }
        const result = mapReducer(state, action)
        expect(result).toEqual(DEFAULT_MAP_DATA)
    });
 })