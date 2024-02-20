import { MARKER, PERIMETER } from "../support/elements";

const TEST_INPUT = {
  desc: 'test-marker',
  coords: { lat: 36.68, lng: -119.8 },
};

describe('template spec', () => {
  beforeEach(() => {
    cy.visit('/');
    cy.contains('Splot');
  });

  it('should add new marker to map', () => {
    cy.get(MARKER.INPUT.LAT).type(TEST_INPUT.coords.lat.toString());
    cy.get(MARKER.INPUT.LNG).type(TEST_INPUT.coords.lng.toString());
    cy.get(MARKER.INPUT.DESC).type(TEST_INPUT.desc);
    cy.get(MARKER.BUTTON.SUBMIT).click();

    cy.contains(TEST_INPUT.desc);
    cy.get(MARKER.BUTTON.DELETE).last().click();
    cy.contains(TEST_INPUT.desc).should('not.exist');
  });

  it('should add new perimeter to map', () => {
    cy.get(PERIMETER.INPUT.LAT).type(TEST_INPUT.coords.lat.toString());
    cy.get(PERIMETER.INPUT.LNG).type(TEST_INPUT.coords.lng.toString());
    cy.get(PERIMETER.INPUT.DESC).type(TEST_INPUT.desc);
    cy.get(PERIMETER.BUTTON.SUBMIT).click();

    cy.contains(TEST_INPUT.desc);
    cy.get(PERIMETER.BUTTON.DELETE).last().click();
    cy.contains(TEST_INPUT.desc).should('not.exist');
  });
});
