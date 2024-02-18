import { render, screen } from "@testing-library/react";
import MapMarkerList from "../MapMarkerList";
import { MAP_MARKERS } from "../../constants";

describe("MapMarkerList", () => {
  it("should show all the markers for mock data", () => {
    render(<MapMarkerList markers={MAP_MARKERS} />);
    const numberOfListItems = screen.queryAllByRole("listitem").length;
    expect(numberOfListItems).toBe(MAP_MARKERS.length);
  });

  it("should show no markers for empty data", () => {
    render(<MapMarkerList markers={[]} />);
    const numberOfListItems = screen.queryAllByRole("listitem").length;
    expect(numberOfListItems).toBe(0);
  });
});
