import { useState, useEffect } from 'react';
import styles from './styles/SystemClock.module.css';

const SystemClock = () => {
  const [time, setTime] = useState(new Date());

  useEffect(() => {
    const timer = setInterval(() => setTime(new Date()), 1000);
    return () => clearInterval(timer);
  }, []);

  return (
    <div className={styles.container}>
      SİSTEM SAATİ: <span className={styles.timeValue}>{time.toLocaleTimeString()}</span>
    </div>
  );
};

export default SystemClock;