export const buildMissionWaypoints = (points) => {
  if (!Array.isArray(points) || points.length === 0) {
    throw new Error('Görev listesi boş.');
  }
  const normalized = points.map((point, index) => {
    const lat = Number(point?.lat);
    const lon = Number(point?.lon);
    const parkur = Number(point?.parkur);
    if (!Number.isFinite(lat) || lat < -90 || lat > 90 ||
        !Number.isFinite(lon) || lon < -180 || lon > 180 ||
        !Number.isInteger(parkur) || parkur < 1 || parkur > 3) {
      throw new Error(`WP#${index + 1} koordinat/parkur değeri geçersiz.`);
    }
    return { lat, lon, alt: 0, parkur };
  });
  const parkurs = normalized.map((point) => point.parkur);
  if (parkurs[0] !== 1 || parkurs.some((value, index) => index > 0 && value < parkurs[index - 1])) {
    throw new Error('Parkur sırası P1 ile başlamalı ve P1 → P2 → P3 olmalıdır.');
  }
  return normalized;
};
