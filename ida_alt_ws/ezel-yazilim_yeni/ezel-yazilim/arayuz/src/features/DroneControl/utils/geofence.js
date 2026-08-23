/**
 * İHA uçuş bölgesi (geofence) yardımcıları.
 *
 * Şartname §5.5.3.1: "İHA'nın uçuş bölgesi kıyı tarafında belirlenecek alan ile
 * kısıtlı olacaktır. Bu alan dışında ya da deniz tarafında uçuş yapılması
 * durumunda Parkur 3 görevi başarısız sayılacaktır."
 *
 * Sınırı hakemler yarışma günü brifingde bildirir; operatör YKİ'de poligon olarak
 * tanımlar. Buradaki fonksiyonlar saf (yan etkisiz) tutulmuştur.
 */

/** Geçerli bir poligon için en az 3 nokta gerekir. */
export const MIN_GEOFENCE_POINTS = 3;

const isFiniteCoord = (value) => Number.isFinite(value);

/**
 * Noktanın poligon içinde olup olmadığını ray-casting ile hesaplar.
 *
 * @returns {boolean|null} Sınır tanımlı değilse (3'ten az nokta) veya konum
 *   geçersizse `null` döner. `null` "bilinmiyor" demektir ve "ihlal" ile
 *   karıştırılmamalıdır — çağıran taraf bu üç durumu ayrı göstermelidir.
 */
export const isInsidePolygon = (lat, lon, points) => {
  if (!Array.isArray(points) || points.length < MIN_GEOFENCE_POINTS) return null;
  if (!isFiniteCoord(lat) || !isFiniteCoord(lon)) return null;
  // (0,0) GPS fix'i yokken gelen varsayılan değerdir, gerçek konum değildir.
  if (lat === 0 && lon === 0) return null;

  let inside = false;
  for (let i = 0, j = points.length - 1; i < points.length; j = i++) {
    const xi = points[i].lon;
    const yi = points[i].lat;
    const xj = points[j].lon;
    const yj = points[j].lat;
    if (!isFiniteCoord(xi) || !isFiniteCoord(yi) || !isFiniteCoord(xj) || !isFiniteCoord(yj)) {
      return null;
    }

    const intersects = (yi > lat) !== (yj > lat)
      && lon < ((xj - xi) * (lat - yi)) / (yj - yi) + xi;
    if (intersects) inside = !inside;
  }
  return inside;
};

/** Poligonun ağırlık merkezi — haritayı bölgeye odaklamak için. */
export const polygonCentroid = (points) => {
  if (!Array.isArray(points) || points.length === 0) return null;
  const valid = points.filter((p) => isFiniteCoord(p?.lat) && isFiniteCoord(p?.lon));
  if (valid.length === 0) return null;

  const sum = valid.reduce(
    (acc, p) => ({ lat: acc.lat + p.lat, lon: acc.lon + p.lon }),
    { lat: 0, lon: 0 },
  );
  return { lat: sum.lat / valid.length, lon: sum.lon / valid.length };
};

/** Leaflet `Polygon` bileşeninin beklediği [[lat, lon], ...] biçimi. */
export const toLatLngPairs = (points) => (
  (points || [])
    .filter((p) => isFiniteCoord(p?.lat) && isFiniteCoord(p?.lon))
    .map((p) => [p.lat, p.lon])
);
