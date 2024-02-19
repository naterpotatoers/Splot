describe('template spec', () => {
  const marker = { desc: 'test-marker', coords: { lat: 36.68, lng: -119.7 } };
  const LAT_INPUT = 'input[name="lat"]';
  const LNG_INPUT = 'input[name="lng"]';
  const DESC_INPUT = 'input[name="desc"]';
  const SUBMIT_BTN = 'button[type="submit"]';

  it('should add new marker to map', () => {
    cy.visit('/');
    cy.contains('Splot');

    cy.get(LAT_INPUT).type(marker.coords.lat);
    cy.get(LNG_INPUT).type(marker.coords.lng);
    cy.get(DESC_INPUT).type(marker.desc);
    cy.get(SUBMIT_BTN).click();

    cy.contains(marker.desc);
    cy.get('ol li').last().find('button').click();
    cy.contains(marker.desc).should('not.exist');
  });
});
