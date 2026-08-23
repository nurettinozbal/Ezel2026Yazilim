// src/services/simulation/mockDataGenerator.js

const BASE_LAT = 41.025;
const BASE_LON = 28.974;

export const generateMockData = () => {
  // Titreşim efekti
  const noise = (Math.random() - 0.5) * 0.0001;

  return {
    ida: {
      sys_id: 1,
      mode: "AUTO",
      battery_percent: Math.floor(Math.random() * (100 - 80) + 80),
      voltage: parseFloat((16.0 + Math.random()).toFixed(1)),
      current: parseFloat((12.0 + Math.random() * 5).toFixed(1)),
      speed: parseFloat((1.5 + Math.random()).toFixed(1)),
      lat: BASE_LAT + noise,
      lon: BASE_LON + noise,
      heading: Math.floor(Math.random() * 360),
      target_heading: Math.floor(Math.random() * 360),
      target_speed: parseFloat((1.7 + Math.random()).toFixed(1)),
      current_wp: 2,
      dist_to_wp: parseFloat((15.4 + Math.random()).toFixed(1)),
      motor_left_pwm: 1500 + Math.floor(Math.random() * 250),
      motor_right_pwm: 1500 + Math.floor(Math.random() * 250),
      motor_left_pct: parseFloat((Math.random() * 50).toFixed(1)),
      motor_right_pct: parseFloat((Math.random() * 50).toFixed(1)),
      rpm_left: null,
      rpm_right: null,
      roll: parseFloat(((Math.random() - 0.5) * 0.1).toFixed(3)),
      pitch: parseFloat(((Math.random() - 0.5) * 0.1).toFixed(3)),
      yaw: parseFloat((Math.random() * 6.28).toFixed(3)),
      home_lat: BASE_LAT,
      home_lon: BASE_LON,
      home_locked: true,
      home_locked_at: Date.now() / 1000,
      return_home_pending: false,
      return_home_status: "idle",
      return_home_message: "",
      armed: true,
      connected: true,
      last_heartbeat: Date.now() / 1000,
    },
    
    iha: {
      battery: 85,
      alt: 12.5,
      detected_color: ["KIRMIZI", "YEŞİL", "SİYAH", "YOK"][Math.floor(Math.random() * 4)],
      last_update: Date.now(),
      // --- EKSİK OLAN KISIM BURASIYDI ---
      lat: BASE_LAT + 0.001 + noise, // İHA biraz kuzeyde olsun
      lon: BASE_LON + 0.001 + noise,
      home_lat: BASE_LAT + 0.001,
      home_lon: BASE_LON + 0.001,
      home_locked: true,
      home_locked_at: Date.now() / 1000,
      return_home_pending: false,
      return_home_status: "idle",
      return_home_message: "",
      armed: false,
      connected: true,
    },

    system: {
      rssi: Math.floor(-50 + Math.random() * 10),
      gps_sats: 14,
      hdop: parseFloat((0.7 + Math.random() * 0.2).toFixed(1)),
      failsafe_status: "RTL_READY",
      ida_link: "CONNECTED",
      iha_link: "CONNECTED",
      auto_return_on_link_loss: true,
      return_home_mode: "RTL",
      logs: [],
    }
  };
};
