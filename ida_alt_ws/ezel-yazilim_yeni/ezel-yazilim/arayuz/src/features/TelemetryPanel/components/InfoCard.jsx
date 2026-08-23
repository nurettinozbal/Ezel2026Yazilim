import styles from '../styles/Telemetry.module.css';

const InfoCard = ({ title, value, unit, icon: Icon, color = "#3498db", alert = false }) => {
  return (
    <div className={`${styles.card} ${alert ? styles.alertMode : ''}`}>
      <div className={styles.iconContainer} style={{ backgroundColor: `${color}20`, color: color }}>
        {/* İkon prop olarak geldiyse (<Icon />) şeklinde render et */}
        {Icon && <Icon size={24} />}
      </div>
      <div className={styles.dataContainer}>
        <span className={styles.cardTitle}>{title}</span>
        <div className={styles.valueRow}>
          <span className={styles.cardValue}>{value}</span>
          <span className={styles.cardUnit}>{unit}</span>
        </div>
      </div>
    </div>
  );
};

export default InfoCard;