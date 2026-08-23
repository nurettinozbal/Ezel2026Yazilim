import { generateMockData } from './simulation/mockDataGenerator';

const DEFAULT_BACKEND_URL = 'ws://localhost:5000/ws';
const RECONNECT_DELAY_MS = 2000;
const DISCONNECT_GRACE_MS = 350;

const parseBooleanEnv = (value) => {
  if (typeof value !== 'string') return false;
  return ['1', 'true', 'yes', 'on'].includes(value.trim().toLowerCase());
};

const normalizeWebSocketUrl = (rawUrl) => {
  const fallbackUrl = DEFAULT_BACKEND_URL;
  const trimmedUrl = typeof rawUrl === 'string' ? rawUrl.trim() : '';
  const configuredUrl = trimmedUrl || fallbackUrl;

  try {
    const parsed = new URL(configuredUrl, window.location.origin);
    const isLocalHost = ['localhost', '127.0.0.1'].includes(parsed.hostname);
    if (parsed.protocol === 'http:') parsed.protocol = 'ws:';
    if (parsed.protocol === 'https:') parsed.protocol = isLocalHost ? 'ws:' : 'wss:';
    if (parsed.protocol === 'wss:' && isLocalHost) {
      console.warn(
        `[TelemetryService] Güvenli olmayan yerel backend beklendiği için WebSocket protokolü ws:// olarak zorlandı. Configured=${configuredUrl}`,
      );
      parsed.protocol = 'ws:';
    }
    return parsed.toString();
  } catch (error) {
    console.warn(
      `[TelemetryService] Geçersiz VITE_WS_URL algılandı, varsayılana dönülüyor. Configured=${configuredUrl}`,
      error,
    );
    return fallbackUrl;
  }
};

const buildLocalhostFallbackUrl = (rawUrl) => {
  try {
    const parsed = new URL(rawUrl);
    if (parsed.hostname !== 'localhost') return null;
    parsed.hostname = '127.0.0.1';
    return parsed.toString();
  } catch {
    return null;
  }
};

class TelemetryService {
  constructor() {
    this.socket = null;
    this.intervalId = null;
    this.reconnectTimerId = null;
    this.disconnectTimerId = null;
    this.shouldReconnect = false;
    this.activeUrl = null;
    this.fallbackUrl = null;
    this.authToken = '';
    this.commandAuthorized = false;
    this.useMock = false;
    this.handlers = {
      onTelemetry: () => {},
      onLog: () => {},
      onStatus: () => {},
      onCommandResult: () => {},
      onDebugSnapshot: () => {},
      onVehicleTest: () => {},
    };
  }

  connect({ onTelemetry, onLog, onStatus, onCommandResult, onDebugSnapshot, onVehicleTest }) {
    this.cancelPendingDisconnect();
    this.stopMockStream();
    this.clearReconnectTimer();
    this.handlers = {
      onTelemetry: onTelemetry || (() => {}),
      onLog: onLog || (() => {}),
      onStatus: onStatus || (() => {}),
      onCommandResult: onCommandResult || (() => {}),
      onDebugSnapshot: onDebugSnapshot || (() => {}),
      onVehicleTest: onVehicleTest || (() => {}),
    };
    this.shouldReconnect = true;

    const useMock = parseBooleanEnv(import.meta.env.VITE_USE_MOCK);
    const backendUrl = normalizeWebSocketUrl(import.meta.env.VITE_WS_URL);
    const wsToken = typeof import.meta.env.VITE_WS_TOKEN === 'string'
      ? import.meta.env.VITE_WS_TOKEN.trim()
      : '';
    this.useMock = useMock;
    this.activeUrl = backendUrl;
    this.fallbackUrl = buildLocalhostFallbackUrl(backendUrl);
    this.authToken = wsToken;
    this.commandAuthorized = false;

    if (useMock) {
      console.info('[TelemetryService] Frontend transport mode: MOCK');
      this.handlers.onLog({
        level: 'INFO',
        message: 'Frontend transport mode: MOCK (VITE_USE_MOCK aktif)',
      });
      this.handlers.onStatus('MOCK');
      this.handlers.onTelemetry(generateMockData());
      this.intervalId = setInterval(() => {
        this.handlers.onTelemetry(generateMockData());
      }, 500);
      return;
    }

    console.info(
      `[TelemetryService] Frontend transport mode: BACKEND | url=${backendUrl} | VITE_WS_URL=${import.meta.env.VITE_WS_URL || '(default)'} | VITE_USE_MOCK=${import.meta.env.VITE_USE_MOCK || '(unset)'}`,
    );
    this.handlers.onLog({
      level: 'INFO',
      message: `Frontend transport mode: BACKEND | WebSocket=${backendUrl}`,
    });
    if (!wsToken) {
      this.handlers.onLog({
        level: 'WARNING',
        message: 'VITE_WS_TOKEN ayarlı değil; backend komut auth aktifse araç komutları reddedilir.',
      });
    }

    if (this.socket && (this.socket.readyState === WebSocket.OPEN || this.socket.readyState === WebSocket.CONNECTING)) {
      this.handlers.onStatus(this.socket.readyState === WebSocket.OPEN ? 'CONNECTED' : 'CONNECTING');
      return;
    }

    this.connectWebSocket(backendUrl);
  }

  connectWebSocket(url = this.activeUrl || DEFAULT_BACKEND_URL) {
    if (!this.shouldReconnect) return;

    this.handlers.onStatus('CONNECTING');

    let socket;
    let opened = false;
    try {
      socket = new WebSocket(url);
      this.socket = socket;
    } catch (error) {
      console.error(`[TelemetryService] WebSocket bağlantısı oluşturulamadı: ${url}`, error);
      this.handlers.onStatus('ERROR');
      this.scheduleReconnect();
      return;
    }

    socket.onopen = () => {
      if (this.socket !== socket) return;
      opened = true;
      this.handlers.onStatus('CONNECTED');
      this.authenticateSocket(socket);
    };

    socket.onmessage = (event) => {
      if (this.socket !== socket) return;
      this.handleMessage(event.data);
    };

    socket.onerror = (event) => {
      if (this.socket !== socket) return;
      console.warn(`[TelemetryService] WebSocket hata olayı alındı: ${url}`, event);
      this.handlers.onStatus('ERROR');
    };

    socket.onclose = (event) => {
      if (this.socket !== socket) return;
      this.socket = null;
      console.warn(
        `[TelemetryService] WebSocket kapandı: ${url} | code=${event.code} reason=${event.reason || '(none)'}`,
      );

      if (this.shouldReconnect) {
        if (!opened && this.fallbackUrl && url !== this.fallbackUrl) {
          console.warn(`[TelemetryService] localhost bağlantısı başarısız; fallback deneniyor: ${this.fallbackUrl}`);
          this.scheduleReconnect(250, this.fallbackUrl);
        } else {
          this.scheduleReconnect(RECONNECT_DELAY_MS, this.activeUrl);
        }
      } else {
        this.handlers.onStatus('DISCONNECTED');
      }
    };
  }

  authenticateSocket(socket) {
    if (!this.authToken) return;
    try {
      socket.send(JSON.stringify({
        type: 'auth',
        token: this.authToken,
      }));
    } catch (error) {
      console.error('[TelemetryService] Komut auth mesajı gönderilemedi', error);
    }
  }

  handleMessage(rawMessage) {
    let message;

    try {
      message = JSON.parse(rawMessage);
    } catch (error) {
      console.error('Geçersiz WebSocket mesajı:', error);
      return;
    }

    const body = message.payload ?? message.data;

    if (message.type === 'telemetry') {
      const telemetry = body ?? message.telemetry;
      if (telemetry && typeof telemetry === 'object') {
        this.handlers.onTelemetry(telemetry);
      }
      return;
    }

    if (message.type === 'log') {
      this.handlers.onLog(body ?? message);
      return;
    }

    if (message.type === 'command_result') {
      this.handlers.onCommandResult(body ?? message);
      return;
    }

    if (message.type === 'debug_snapshot') {
      if (body && typeof body === 'object' && body.schema_version === 1) {
        this.handlers.onDebugSnapshot(body);
      }
      return;
    }

    if (message.type === 'vehicle_test') {
      if (body && typeof body === 'object' && body.schema_version === 1) {
        this.handlers.onVehicleTest(body);
      }
      return;
    }

    if (message.type === 'auth') {
      const auth = body ?? message;
      this.commandAuthorized = Boolean(auth.command_authorized);
      this.handlers.onLog({
        level: this.commandAuthorized ? 'SUCCESS' : 'WARNING',
        message: auth.message || (this.commandAuthorized ? 'Komut auth başarılı' : 'Komut auth bekleniyor/başarısız'),
      });
    }
  }

  scheduleReconnect(delay = RECONNECT_DELAY_MS, url = this.activeUrl) {
    if (!this.shouldReconnect || this.reconnectTimerId) return;

    this.handlers.onStatus('RECONNECTING');
    this.reconnectTimerId = setTimeout(() => {
      this.reconnectTimerId = null;
      this.connectWebSocket(url || this.activeUrl);
    }, delay);
  }

  sendCommand(command, payload = {}) {
    if (this.useMock) {
      this.handlers.onLog({
        level: 'INFO',
        message: `Mock komut simüle edildi: ${command}`,
      });
      this.handlers.onCommandResult({
        ok: true,
        command,
        message: `Mock komut başarılı: ${command}`,
      });
      return true;
    }

    if (!this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return false;
    }

    try {
      this.socket.send(JSON.stringify({
        type: 'command',
        command,
        payload,
      }));
      return true;
    } catch (error) {
      console.error(`Komut gönderilemedi: ${command}`, error);
      return false;
    }
  }

  sendVehicleTest(action, payload = {}) {
    if (this.useMock || !this.socket || this.socket.readyState !== WebSocket.OPEN) {
      return false;
    }
    try {
      this.socket.send(JSON.stringify({
        type: 'vehicle_test',
        data: { action, ...payload },
      }));
      return true;
    } catch (error) {
      console.error('Pasif test istegi gonderilemedi', error);
      return false;
    }
  }

  disconnect() {
    this.shouldReconnect = false;
    this.stopMockStream();
    this.clearReconnectTimer();

    if (this.socket) {
      const socket = this.socket;
      this.disconnectTimerId = setTimeout(() => {
        if (this.shouldReconnect || this.socket !== socket) return;
        socket.onopen = null;
        socket.onmessage = null;
        socket.onerror = null;
        socket.onclose = null;
        socket.close();
        this.socket = null;
      }, DISCONNECT_GRACE_MS);
    }
  }

  stopMockStream() {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
  }

  clearReconnectTimer() {
    if (this.reconnectTimerId) {
      clearTimeout(this.reconnectTimerId);
      this.reconnectTimerId = null;
    }
  }

  cancelPendingDisconnect() {
    if (this.disconnectTimerId) {
      clearTimeout(this.disconnectTimerId);
      this.disconnectTimerId = null;
    }
  }
}

export const telemetryService = new TelemetryService();
