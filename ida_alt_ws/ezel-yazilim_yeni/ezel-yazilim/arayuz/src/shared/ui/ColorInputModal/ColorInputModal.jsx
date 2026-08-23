import { useState } from 'react';
import styles from './ColorInputModal.module.css';

/**
 * ColorInputModal — GCS uyumlu renk giriş modalı.
 * window.prompt() yerine kullanılır; fullscreen/focus kaybını önler.
 * Davranış aynıdır: kullanıcıdan metin alır, onConfirm ile döner.
 */
const ColorInputModal = ({ isOpen, defaultValue = '', onConfirm, onCancel }) => {
  const [value, setValue] = useState(defaultValue);

  const handleSubmit = (e) => {
    e.preventDefault();
    const trimmed = value.trim();
    if (trimmed) {
      onConfirm(trimmed);
    }
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Escape') {
      onCancel();
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.overlay} onClick={onCancel} onKeyDown={handleKeyDown}>
      <form 
        className={styles.modal} 
        onClick={(e) => e.stopPropagation()} 
        onSubmit={handleSubmit}
      >
        <div className={styles.title}>HEDEF RENK GİRİŞİ</div>
        <div className={styles.subtitle}>
          Tespit edilen rengi giriniz (Örn: KIRMIZI, YEŞİL, SİYAH)
        </div>
        <input
          type="text"
          value={value}
          onChange={(e) => setValue(e.target.value)}
          className={styles.input}
          placeholder="KIRMIZI"
          autoFocus
          autoComplete="off"
          spellCheck="false"
        />
        <div className={styles.actions}>
          <button type="button" onClick={onCancel} className={styles.cancelButton}>
            İPTAL
          </button>
          <button type="submit" className={styles.confirmButton}>
            ONAYLA
          </button>
        </div>
      </form>
    </div>
  );
};

export default ColorInputModal;
