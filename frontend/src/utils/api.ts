import axios from 'axios';

export async function getExploredPositions() {
  const response = await axios.get('http://localhost:5000/splot/explored');
  return response.data;
}
