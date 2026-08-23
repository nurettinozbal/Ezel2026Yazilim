import styles from './ConfirmModal.module.css';

/**
 * ConfirmModal — geri alınamaz/riskli işlemler için onay modalı.
 * ColorInputModal ile aynı GCS görsel dilini kullanır.
 */
const ConfirmModal = ({ isOpen, title, message, confirmLabel = 'ONAYLA', onConfirm, onCancel }) => {
  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={onCancel}>
      <div className={styles.modal} onClick={(e) => e.stopPropagation()}>
        <div className={styles.title}>{title}</div>
        <div className={styles.message}>{message}</div>
        <div className={styles.actions}>
          <button type="button" onClick={onCancel} className={styles.cancelButton}>
            İPTAL
          </button>
          <button type="button" onClick={onConfirm} className={styles.confirmButton}>
            {confirmLabel}
          </button>
        </div>
      </div>
    </div>
  );
};

export default ConfirmModal;
