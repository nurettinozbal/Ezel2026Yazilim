import { useEffect, useRef } from 'react';
import { useVehicle } from '@/context/useVehicle';
import { Terminal } from 'lucide-react';
import styles from '../styles/LogTerminal.module.css'; // Yeni CSS Modülü

const LogTerminal = ({ height = '150px' }) => {
  const { systemLogs } = useVehicle();
  const logsEndRef = useRef(null);

  useEffect(() => {
    logsEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [systemLogs]);

  const getColor = (type) => {
    switch (type) {
      case 'SUCCESS': return '#2ecc71';
      case 'WARNING': return '#f1c40f';
      case 'ERROR': return '#ef4444';
      default: return '#3498db';
    }
  };

  return (
    <div className={styles.container} style={{ height: height }}>
      
      {/* BAŞLIK */}
      <div className={styles.header}>
        <Terminal size={14} /> SİSTEM LOGLARI & TERMİNAL
      </div>

      {/* LOG LİSTESİ */}
      <div className={styles.logArea}>
        {systemLogs.map((log) => (
          <div key={log.id} className={styles.logRow}>
            <span className={styles.timestamp}>[{log.time}]</span>
            <span 
              className={styles.message}
              style={{ color: getColor(log.type), fontWeight: log.type === 'ERROR' ? 'bold' : 'normal' }}
            >
              {log.type === 'INFO' ? '>' : log.type}: {log.msg}
            </span>
          </div>
        ))}
        
        {systemLogs.length === 0 && <span className={styles.emptyMsg}>Sistem hazır, log bekleniyor...</span>}
        
        <div ref={logsEndRef} />
      </div>

    </div>
  );
};

export default LogTerminal;
