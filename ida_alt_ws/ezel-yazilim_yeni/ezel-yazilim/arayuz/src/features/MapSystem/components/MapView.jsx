import React, { useRef, useEffect, useState, useCallback } from 'react';
import { MapContainer, TileLayer, Marker, Popup, Polyline, Polygon, CircleMarker, useMapEvents } from 'react-leaflet';
import 'leaflet/dist/leaflet.css';
import styles from '../styles/MapSystem.module.css';
import { useVehicle } from '@/context/useVehicle';
import { toLatLngPairs, MIN_GEOFENCE_POINTS } from '@/features/DroneControl/utils/geofence';
import L from 'leaflet';

// --- TAKTİKSEL İKON TANIMLARI ---
const ykiIcon = L.divIcon({
  className: 'custom-purple-icon',
  html: `<div style="background-color: #9b59b6; width: 16px; height: 16px; border-radius: 50%; border: 3px solid #fff; box-shadow: 0 0 15px #9b59b6; position: relative;">
           <div style="position: absolute; top: -22px; left: -32px; color: #9b59b6; font-weight: bold; font-size: 11px; text-shadow: 1px 1px 3px #000; width: 80px; text-align: center;">YKİ MERKEZ</div>
         </div>`,
  iconSize: [22, 22],
  iconAnchor: [11, 11],
});

const idaIcon = L.divIcon({
  className: 'custom-red-icon', 
  html: '<div class="pulse-ring"></div><div class="inner-dot"></div>',
  iconSize: [24, 24],
  iconAnchor: [12, 12], 
});

const ihaIcon = L.divIcon({
  className: 'custom-orange-icon',
  html: '<div class="pulse-ring-orange"></div><div class="inner-dot-orange"></div>',
  iconSize: [24, 24],
  iconAnchor: [12, 12],
});

// --- NUMARALI VE RENKLİ GÖREV NOKTASI İKONU ---
const createNumberedIcon = (text, isStart = false) => {
  const bgColor = isStart ? '#3498db' : '#f1c40f'; // Başlangıç noktası mavi, hedefler sarı
  const textColor = isStart ? '#fff' : '#000';
  return L.divIcon({
    className: 'numbered-icon',
    html: `<div style="background-color: ${bgColor}; color: ${textColor}; width: 26px; height: 26px; display: flex; align-items: center; justify-content: center; border-radius: 50%; border: 2px solid #fff; box-shadow: 0 0 8px rgba(0,0,0,0.5); font-weight: bold; font-size: 12px;">${text}</div>`,
    iconSize: [26, 26],
    iconAnchor: [13, 13],
    popupAnchor: [0, -13],
  });
};

const isValidPosition = (lat, lon) => (
  Number.isFinite(lat) && Number.isFinite(lon) &&
  lat >= -90 && lat <= 90 && lon >= -180 && lon <= 180 &&
  !(lat === 0 && lon === 0)
);

// --- HARİTA DİNLEYİCİSİ (AKILLI HOME NOKTASI EKLENDİ) ---
function SimpleMapClickHandler() {
  const { addMissionPoint, missionPoints, telemetry } = useVehicle();
  const ida = telemetry?.ida;

  useMapEvents({
    click(e) {
      const lat = parseFloat(e.latlng.lat.toFixed(6));
      const lon = parseFloat(e.latlng.lng.toFixed(6));

      if (missionPoints.length === 0) {
        // Liste boşken ilk tıklama yapıldığında, Pixhawk 1. noktayı atlamasın diye
        // ÖNCE aracın mevcut konumunu "HOME (Başlangıç)" olarak ekliyoruz.
        const startLat = (ida && ida.lat !== 0) ? ida.lat : lat;
        const startLon = (ida && ida.lon !== 0) ? ida.lon : lon;
        
        // Mission contract always starts at P1. The clicked target that follows
        // uses the operator-selected default parkur from VehicleContext.
        addMissionPoint(startLat, startLon, 1);

        // Eğer Home konumu olarak aracın konumu başarıyla eklendiyse,
        // asıl gitmek istediğimiz tıklanan yeri de "1. Hedef" olarak peşinden ekliyoruz.
        if (startLat !== lat || startLon !== lon) {
          setTimeout(() => {
            addMissionPoint(lat, lon);
          }, 50); // React state'inin karışmaması için 50ms gecikme
        }
      } else {
        // Zaten görev varsa, sadece tıklanan yeri ekle
        addMissionPoint(lat, lon);
      }
    },
  });
  return null; 
}

// --- GEOFENCE ÇİZİM DİNLEYİCİSİ ---
// Görev noktası ekleyen SimpleMapClickHandler ile aynı desende, ama tıklamayı
// İHA uçuş bölgesi sınırına yönlendirir. İkisi aynı anda aktif olmamalıdır.
function GeofenceClickHandler() {
  const { addGeofencePoint } = useVehicle();

  useMapEvents({
    click(e) {
      addGeofencePoint(
        parseFloat(e.latlng.lat.toFixed(6)),
        parseFloat(e.latlng.lng.toFixed(6)),
      );
    },
  });
  return null;
}

const MapView = ({ planningEnabled = false, geofenceMode = false }) => {
  const { telemetry, missionPoints, geofencePoints, isIhaInsideGeofence } = useVehicle();
  const { ida, iha } = telemetry;
  const geofenceLatLngs = toLatLngPairs(geofencePoints);
  const geofenceComplete = geofenceLatLngs.length >= MIN_GEOFENCE_POINTS;
  // İhlal sadece sınır tanımlıyken anlamlıdır; null "bilinmiyor" demektir.
  const geofenceViolated = geofenceComplete && isIhaInsideGeofence === false;

  const [ykiLocation, setYkiLocation] = useState(null);
  const mapRef = useRef(null);

  // Tarayıcıdan YKİ konumunu al ve Haritayı Oraya Uçur
  useEffect(() => {
    if ("geolocation" in navigator) {
      navigator.geolocation.getCurrentPosition(
        (position) => {
          const newLocation = [position.coords.latitude, position.coords.longitude];
          setYkiLocation(newLocation);
          
          if (mapRef.current) {
            mapRef.current.flyTo(newLocation, 18, { animate: true, duration: 1.5 });
          }
        },
        (error) => {
          console.warn("YKİ konumu alınamadı. Harita aracın konumunda kalacak.", error.message);
        },
        { enableHighAccuracy: true, timeout: 10000, maximumAge: 0 }
      );
    }
  }, []);

  const routeLine = missionPoints.map(p => [p.lat, p.lon]);
  const hasCenteredOnIdaRef = useRef(isValidPosition(ida.lat, ida.lon));
  const idaPositionValid = isValidPosition(ida.lat, ida.lon);
  const ihaPositionValid = isValidPosition(iha.lat, iha.lon);
  const idaHomeValid = isValidPosition(ida.home_lat, ida.home_lon);
  const ihaHomeValid = isValidPosition(iha.home_lat, iha.home_lon);
  
  const initialCenter = idaPositionValid ? [ida.lat, ida.lon] : (ykiLocation || [39.925533, 32.866287]);

  const [sysWarning, setSysWarning] = useState("");

  const triggerWarning = useCallback((msg) => {
    setSysWarning(msg);
    setTimeout(() => setSysWarning(""), 3000);
  }, []);

  const focusOnIda = useCallback(() => {
    if (mapRef.current) {
      if (isValidPosition(ida.lat, ida.lon)) mapRef.current.flyTo([ida.lat, ida.lon], 18, { animate: true, duration: 1.5 });
      else triggerWarning("⚠️ İDA (GEMİ) için geçerli GPS verisi bulunamadı!");
    }
  }, [ida.lat, ida.lon, triggerWarning]);

  const focusOnIha = useCallback(() => {
    if (mapRef.current) {
      if (isValidPosition(iha.lat, iha.lon)) mapRef.current.flyTo([iha.lat, iha.lon], 18, { animate: true, duration: 1.5 });
      else triggerWarning("⚠️ İHA (DRONE) için geçerli GPS verisi bulunamadı!");
    }
  }, [iha.lat, iha.lon, triggerWarning]);

  const focusOnYki = useCallback(() => {
    if (mapRef.current) {
      if (ykiLocation) {
        mapRef.current.flyTo(ykiLocation, 18, { animate: true, duration: 1.5 });
      } else {
        triggerWarning("⚠️ YKİ (Bilgisayar) GPS konumu henüz alınamadı!");
      }
    }
  }, [ykiLocation, triggerWarning]);

  useEffect(() => {
    window.addEventListener('FOCUS_IDA', focusOnIda);
    window.addEventListener('FOCUS_IHA', focusOnIha);
    window.addEventListener('FOCUS_YKI', focusOnYki);
    return () => {
      window.removeEventListener('FOCUS_IDA', focusOnIda);
      window.removeEventListener('FOCUS_IHA', focusOnIha);
      window.removeEventListener('FOCUS_YKI', focusOnYki);
    };
  }, [focusOnIda, focusOnIha, focusOnYki]);

  useEffect(() => {
    if (!mapRef.current || hasCenteredOnIdaRef.current || !idaPositionValid) return;
    mapRef.current.setView([ida.lat, ida.lon], 17, { animate: false });
    hasCenteredOnIdaRef.current = true;
  }, [ida.lat, ida.lon, idaPositionValid]);

  const renderDistanceLabels = () => {
    const labels = [];
    for (let i = 0; i < missionPoints.length - 1; i++) {
      const p1 = missionPoints[i];
      const p2 = missionPoints[i + 1];
      const distance = L.latLng(p1.lat, p1.lon).distanceTo(L.latLng(p2.lat, p2.lon));
      const midLat = (p1.lat + p2.lat) / 2;
      const midLon = (p1.lon + p2.lon) / 2;
      const distText = distance > 1000 ? (distance / 1000).toFixed(2) + ' km' : Math.round(distance) + ' m';

      const distanceIcon = L.divIcon({
        className: 'custom-distance-icon',
        html: `<div style="background-color: #0a0a0a; color: #f1c40f; border: 1px solid #f1c40f; border-radius: 4px; padding: 2px 6px; font-size: 10px; font-weight: bold; white-space: nowrap; box-shadow: 0 2px 4px rgba(0,0,0,0.8); margin-top: -10px;">${distText}</div>`,
        iconSize: [0, 0],
        iconAnchor: [20, 10], 
      });

      labels.push(<Marker key={`dist-${i}`} position={[midLat, midLon]} icon={distanceIcon} interactive={false} />);
    }
    return labels;
  };

  return (
    <div className={styles.mapContainer} style={{ position: 'relative' }}>
      
      {sysWarning && (
        <div style={{
          position: 'absolute', top: '50%', left: '50%', transform: 'translate(-50%, -50%)', 
          backgroundColor: 'rgba(220, 38, 38, 0.95)', color: '#ffffff', padding: '16px 32px', 
          borderRadius: '8px', fontWeight: 'bold', fontSize: '16px', zIndex: 9999, 
          boxShadow: '0 8px 32px rgba(0, 0, 0, 0.6)', border: '2px solid #ff5252',
          letterSpacing: '1px', backdropFilter: 'blur(4px)', animation: 'fadeIn 0.3s ease-out', textAlign: 'center'
        }}>
          {sysWarning}
        </div>
      )}

      <MapContainer 
        ref={mapRef} center={initialCenter} zoom={17} maxZoom={24} scrollWheelZoom={true}
        style={{ height: "100%", width: "100%" }}
      >
        <TileLayer
          url="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
          attribution='Tiles &copy; Esri' maxNativeZoom={19} maxZoom={24}       
        />

        {planningEnabled && !geofenceMode && (
          <SimpleMapClickHandler />
        )}

        {geofenceMode && (
          <GeofenceClickHandler />
        )}

        {/* İHA UÇUŞ BÖLGESİ (ŞARTNAME §5.5.3.1) */}
        {geofenceComplete && (
          <Polygon
            positions={geofenceLatLngs}
            pathOptions={{
              color: geofenceViolated ? '#ef4444' : '#2ecc71',
              weight: 2,
              fillColor: geofenceViolated ? '#ef4444' : '#2ecc71',
              fillOpacity: 0.12,
              dashArray: geofenceViolated ? '6, 6' : undefined,
            }}
          />
        )}

        {/* Sınır henüz tamamlanmadıysa girilen noktaları tek tek göster */}
        {!geofenceComplete && geofenceLatLngs.map((position, index) => (
          <CircleMarker
            key={`geofence-point-${index}`}
            center={position}
            radius={5}
            pathOptions={{ color: '#2ecc71', fillColor: '#2ecc71', fillOpacity: 0.8, weight: 2 }}
          >
            <Popup className="custom-popup">
              <strong style={{ color: '#2ecc71' }}>UÇUŞ BÖLGESİ SINIRI #{index + 1}</strong><br />
              En az {MIN_GEOFENCE_POINTS} nokta gerekli.
            </Popup>
          </CircleMarker>
        ))}

        {ykiLocation && (
          <Marker position={ykiLocation} icon={ykiIcon}>
            <Popup className="custom-popup">
              <strong style={{color: '#9b59b6'}}>EZEL YKİ MERKEZİ</strong><br/>
              Sistem Başlatıldı.<br/> Bağlantı Kuruldu.
            </Popup>
          </Marker>
        )}

        {idaHomeValid && (
          <CircleMarker center={[ida.home_lat, ida.home_lon]} radius={8} color="#38bdf8" fillColor="#38bdf8" fillOpacity={0.35} weight={2}>
            <Popup className="custom-popup">
              <strong style={{color: '#38bdf8'}}>İDA BREAKPOINT</strong><br/>
              Başlangıç / dönüş noktası<br/> {ida.home_lat.toFixed(5)}, {ida.home_lon.toFixed(5)}<br/>
              Durum: {ida.return_home_status || 'idle'}
            </Popup>
          </CircleMarker>
        )}

        {ihaHomeValid && (
          <CircleMarker center={[iha.home_lat, iha.home_lon]} radius={7} color="#f97316" fillColor="#f97316" fillOpacity={0.25} weight={2}>
            <Popup className="custom-popup">
              <strong style={{color: '#f97316'}}>İHA BREAKPOINT</strong><br/>
              Başlangıç / dönüş noktası<br/> {iha.home_lat.toFixed(5)}, {iha.home_lon.toFixed(5)}<br/>
              Durum: {iha.return_home_status || 'idle'}
            </Popup>
          </CircleMarker>
        )}

        {missionPoints.length > 0 && (
          <>
            <Polyline positions={routeLine} color="#f1c40f" weight={3} dashArray="10, 10" opacity={0.8} />
            
            {missionPoints.map((p, index) => {
              const isStart = index === 0;
              const label = isStart ? 'H' : index; // H = Home(Başlangıç)
              
              return (
                <Marker 
                  key={p.id || index} 
                  position={[p.lat, p.lon]} 
                  icon={createNumberedIcon(label, isStart)}
                >
                  <Popup className="custom-popup">
                    {isStart ? (
                      <strong style={{color: '#3498db'}}>BAŞLANGIÇ (HOME) NOKTASI</strong>
                    ) : (
                      <strong style={{color: '#f1c40f'}}>HEDEF #{index}</strong>
                    )}
                    <br/> 
                    Parkur: P{p.parkur ?? 1}<br/>
                    Enlem: {p.lat.toFixed(5)}<br/> 
                    Boylam: {p.lon.toFixed(5)}
                  </Popup>
                </Marker>
              );
            })}
            
            {renderDistanceLabels()}
          </>
        )}

        {/* İDA İkonu: zIndexOffset=1000 ile her zaman en üstte tutuluyor */}
        {idaPositionValid && (
          <Marker position={[ida.lat, ida.lon]} icon={idaIcon} zIndexOffset={1000}>
            <Popup className="custom-popup">
              <strong style={{color: '#ef4444'}}>İDA (GEMİ)</strong><br/>
              Hız: {ida.speed} m/s<br/> Mod: {ida.mode}
            </Popup>
          </Marker>
        )}

        {ihaPositionValid && (
          <Marker position={[iha.lat, iha.lon]} icon={ihaIcon}>
            <Popup className="custom-popup">
              <strong style={{color: '#e67e22'}}>İHA (DRONE)</strong><br/>
              Tespit: {iha.detected_color}<br/> İrtifa: {iha.alt}m
            </Popup>
          </Marker>
        )}
      </MapContainer>
    </div>
  );
};

export default MapView;
